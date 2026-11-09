#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <driver/twai.h>
#include <cmath>
#include <cstring>
#include "BikeCanTelemetry.h"
#include "GeneratedWebPage.h"

// ============================================================
// Hardware and network configuration
// ============================================================
constexpr gpio_num_t CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t CAN_RX = GPIO_NUM_6;
constexpr uint8_t CRANK_NODE_ID = 0;
constexpr uint8_t WHEEL_NODE_ID = 1;
constexpr char AP_SSID[] = "DigitalEBike";
constexpr char AP_PASSWORD[] = "DigitalBike123";   // Change before outdoor use.
constexpr uint32_t STD_FRESH_MS = 2000;
constexpr uint32_t LOG_SAMPLE_MS = 100;            // 10 rows per second
constexpr uint32_t LOG_FLUSH_MS = 1000;
constexpr size_t STORAGE_RESERVE_BYTES = 48 * 1024;

WebServer server(80);
BikeCanTelemetry bike;
bool fsOkay = false;
uint32_t lastAnyCan = 0;
uint32_t standardCanFrames = 0;

struct AxisTelemetry {
    float position = 0, velocity = 0, iq = 0, iqSet = 0;
    uint32_t axisError = 0;
    uint8_t state = 0, motorFlags = 0, encoderFlags = 0, controllerFlags = 0;
    uint32_t lastHeartbeat = 0, lastEncoder = 0, lastIq = 0;
};
AxisTelemetry crank, wheel;
float busVoltage = 0, busCurrent = 0;
uint32_t lastBus = 0;

static bool fresh(uint32_t timestamp, uint32_t timeout = STD_FRESH_MS) {
    return timestamp != 0 && millis() - timestamp < timeout;
}
static float getFloat(const uint8_t* p) {
    float x;
    memcpy(&x, p, sizeof(x));
    return x;
}
static uint32_t getU32(const uint8_t* p) {
    uint32_t x;
    memcpy(&x, p, sizeof(x));
    return x;
}
static void processCan(const twai_message_t& m) {
    if (m.rtr || m.data_length_code != 8) return;
    lastAnyCan = millis();
    if (m.extd) {
        bike.process(m);
        return;
    }
    ++standardCanFrames;
    const uint8_t node = m.identifier >> 5;
    const uint8_t cmd = m.identifier & 0x1F;
    AxisTelemetry* a = node == CRANK_NODE_ID ? &crank :
                      node == WHEEL_NODE_ID ? &wheel : nullptr;
    if (!a) return;
    switch (cmd) {
        case 0x01: // heartbeat
            a->axisError = getU32(&m.data[0]);
            a->state = m.data[4];
            a->motorFlags = m.data[5];
            a->encoderFlags = m.data[6];
            a->controllerFlags = m.data[7];
            a->lastHeartbeat = millis();
            break;
        case 0x09: // encoder estimates, motor turns + turns/s
            a->position = getFloat(&m.data[0]);
            a->velocity = getFloat(&m.data[4]);
            a->lastEncoder = millis();
            break;
        case 0x14: // q-axis current
            a->iqSet = getFloat(&m.data[0]);
            a->iq = getFloat(&m.data[4]);
            a->lastIq = millis();
            break;
        case 0x17: // DC bus (may be received from either axis)
            busVoltage = getFloat(&m.data[0]);
            busCurrent = getFloat(&m.data[4]);
            lastBus = millis();
            break;
    }
}
static bool startCan() {
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    general.rx_queue_len = 64;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
    if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
    return true;
}

// ============================================================
// Single canonical telemetry catalogue for charts and CSV files
// ============================================================
enum MetricId : uint8_t {
    B_CADENCE, B_ROAD, B_WHEEL_RAD, B_RIDER_TORQUE, B_RIDER_POWER,
    B_GEAR, B_TARGET_GEAR, B_CRANK_TORQUE, B_WHEEL_TORQUE,
    B_BATTERY_POWER, B_BRAKE_POWER, B_LIMIT_FACTOR, B_STATE, B_MODE,
    B_BATTERY_LIMITED, B_GENERATOR_LIMITED,
    C_POS, C_VEL, C_IQ, C_IQ_SET, C_STATE, C_ERROR,
    C_MOTOR_FLAGS, C_ENCODER_FLAGS, C_CONTROLLER_FLAGS,
    W_POS, W_VEL, W_IQ, W_IQ_SET, W_STATE, W_ERROR,
    W_MOTOR_FLAGS, W_ENCODER_FLAGS, W_CONTROLLER_FLAGS,
    BUS_VOLTAGE, BUS_CURRENT, METRIC_COUNT
};
struct MetricDef {
    const char* key;
    const char* label;
    const char* unit;
    const char* group;
    bool selectedByDefault;
};
const MetricDef METRICS[] = {
    {"bike.cadence", "Cadence", "rpm", "Bicycle", true},
    {"bike.roadSpeed", "Road speed", "km/h", "Bicycle", true},
    {"bike.wheelRadS", "Wheel angular speed", "rad/s", "Bicycle", false},
    {"bike.riderTorque", "Estimated rider torque", "Nm", "Rider", true},
    {"bike.riderPower", "Estimated rider power", "W", "Rider", true},
    {"bike.gearRatio", "Active virtual gear ratio", "ratio", "Drivetrain", true},
    {"bike.targetGearRatio", "Target virtual gear ratio", "ratio", "Drivetrain", false},
    {"bike.crankTorque", "Crank command torque", "Nm", "Drivetrain", true},
    {"bike.wheelTorque", "Wheel command torque", "Nm", "Drivetrain", true},
    {"bike.batteryPower", "Battery power (+ discharge)", "W", "Power", true},
    {"bike.brakePower", "Brake resistor power", "W", "Power", false},
    {"bike.powerLimitFactor", "Power limit factor", "ratio", "Power", false},
    {"bike.state", "Bike state", "code", "Bike status", false},
    {"bike.mode", "Bike mode", "code", "Bike status", false},
    {"bike.batteryLimited", "Battery discharge limiting", "0/1", "Bike status", false},
    {"bike.generatorLimited", "Generator limiting", "0/1", "Bike status", false},
    {"crank.position", "Crank motor position", "turns", "Crank motor", false},
    {"crank.velocity", "Crank motor velocity", "turns/s", "Crank motor", false},
    {"crank.iq", "Crank measured Iq", "A", "Crank motor", false},
    {"crank.iqSet", "Crank requested Iq", "A", "Crank motor", false},
    {"crank.state", "Crank axis state", "code", "Crank motor", false},
    {"crank.error", "Crank axis error bitmask", "bitmask", "Crank motor", false},
    {"crank.motorFlags", "Crank motor error flags", "bitmask", "Crank motor", false},
    {"crank.encoderFlags", "Crank encoder error flags", "bitmask", "Crank motor", false},
    {"crank.controllerFlags", "Crank controller flags", "bitmask", "Crank motor", false},
    {"wheel.position", "Wheel motor position", "turns", "Wheel motor", false},
    {"wheel.velocity", "Wheel motor velocity", "turns/s", "Wheel motor", false},
    {"wheel.iq", "Wheel measured Iq", "A", "Wheel motor", false},
    {"wheel.iqSet", "Wheel requested Iq", "A", "Wheel motor", false},
    {"wheel.state", "Wheel axis state", "code", "Wheel motor", false},
    {"wheel.error", "Wheel axis error bitmask", "bitmask", "Wheel motor", false},
    {"wheel.motorFlags", "Wheel motor error flags", "bitmask", "Wheel motor", false},
    {"wheel.encoderFlags", "Wheel encoder error flags", "bitmask", "Wheel motor", false},
    {"wheel.controllerFlags", "Wheel controller flags", "bitmask", "Wheel motor", false},
    {"bus.voltage", "DC bus voltage", "V", "DC bus", false},
    {"bus.current", "DC bus current", "A", "DC bus", false}
};
static_assert(sizeof(METRICS)/sizeof(METRICS[0]) == METRIC_COUNT,
              "Metric catalogue and metric IDs do not match");

struct Sample { bool valid; double value; };
static Sample sample(bool valid, double v) {
    return {valid && std::isfinite(v), v};
}
static Sample getMetric(MetricId id) {
    switch (id) {
        case B_CADENCE: return sample(bike.fresh(0), bike.cadenceRpm);
        case B_ROAD: return sample(bike.fresh(0), bike.roadSpeedKmh());
        case B_WHEEL_RAD: return sample(bike.fresh(0), bike.wheelRadS);
        case B_RIDER_TORQUE: return sample(bike.fresh(1), bike.riderTorqueNm);
        case B_RIDER_POWER: return sample(bike.fresh(1), bike.riderPowerW);
        case B_GEAR: return sample(bike.fresh(2), bike.gearRatio);
        case B_TARGET_GEAR: return sample(bike.fresh(2), bike.targetGearRatio);
        case B_CRANK_TORQUE: return sample(bike.fresh(3), bike.crankTorqueNm);
        case B_WHEEL_TORQUE: return sample(bike.fresh(3), bike.wheelTorqueNm);
        case B_BATTERY_POWER: return sample(bike.fresh(4), bike.batteryPowerW);
        case B_BRAKE_POWER: return sample(bike.fresh(4), bike.brakePowerW);
        case B_LIMIT_FACTOR: return sample(bike.fresh(5), bike.powerLimitFactor);
        case B_STATE: return sample(bike.fresh(5), bike.state);
        case B_MODE: return sample(bike.fresh(5), bike.mode);
        case B_BATTERY_LIMITED: return sample(bike.fresh(5), bike.batteryLimited() ? 1 : 0);
        case B_GENERATOR_LIMITED: return sample(bike.fresh(5), bike.generatorLimited() ? 1 : 0);
        case C_POS: return sample(fresh(crank.lastEncoder), crank.position);
        case C_VEL: return sample(fresh(crank.lastEncoder), crank.velocity);
        case C_IQ: return sample(fresh(crank.lastIq), crank.iq);
        case C_IQ_SET: return sample(fresh(crank.lastIq), crank.iqSet);
        case C_STATE: return sample(fresh(crank.lastHeartbeat), crank.state);
        case C_ERROR: return sample(fresh(crank.lastHeartbeat), crank.axisError);
        case C_MOTOR_FLAGS: return sample(fresh(crank.lastHeartbeat), crank.motorFlags);
        case C_ENCODER_FLAGS: return sample(fresh(crank.lastHeartbeat), crank.encoderFlags);
        case C_CONTROLLER_FLAGS: return sample(fresh(crank.lastHeartbeat), crank.controllerFlags);
        case W_POS: return sample(fresh(wheel.lastEncoder), wheel.position);
        case W_VEL: return sample(fresh(wheel.lastEncoder), wheel.velocity);
        case W_IQ: return sample(fresh(wheel.lastIq), wheel.iq);
        case W_IQ_SET: return sample(fresh(wheel.lastIq), wheel.iqSet);
        case W_STATE: return sample(fresh(wheel.lastHeartbeat), wheel.state);
        case W_ERROR: return sample(fresh(wheel.lastHeartbeat), wheel.axisError);
        case W_MOTOR_FLAGS: return sample(fresh(wheel.lastHeartbeat), wheel.motorFlags);
        case W_ENCODER_FLAGS: return sample(fresh(wheel.lastHeartbeat), wheel.encoderFlags);
        case W_CONTROLLER_FLAGS: return sample(fresh(wheel.lastHeartbeat), wheel.controllerFlags);
        case BUS_VOLTAGE: return sample(fresh(lastBus), busVoltage);
        case BUS_CURRENT: return sample(fresh(lastBus), busCurrent);
        default: return {false, 0};
    }
}
static int metricByKey(const String& key) {
    for (int i = 0; i < METRIC_COUNT; ++i) {
        if (key == METRICS[i].key) return i;
    }
    return -1;
}
static void appendNumber(String& target, Sample s) {
    if (!s.valid) { target += "null"; return; }
    char buffer[40];
    snprintf(buffer, sizeof(buffer), "%.15g", s.value);
    target += buffer;
}

// ============================================================
// On-device CSV logger. Fields are frozen for each recording.
// ============================================================
struct CsvLogger {
    File file;
    bool active = false;
    String path;
    String lastError;
    uint8_t ids[METRIC_COUNT] = {};
    uint8_t count = 0;
    uint32_t startedMs = 0;
    uint32_t lastSampleMs = 0;
    uint32_t lastFlushMs = 0;
    uint32_t rows = 0;

    void stop() {
        if (file) {
            file.flush();
            file.close();
        }
        active = false;
    }

    bool start(const String& fields) {
        if (active) { lastError = "A recording is already active"; return false; }
        if (!fsOkay) { lastError = "LittleFS is not mounted"; return false; }
        if (LittleFS.totalBytes() - LittleFS.usedBytes() < STORAGE_RESERVE_BYTES) {
            lastError = "Not enough free flash; download and delete older logs";
            return false;
        }
        count = 0;
        bool seen[METRIC_COUNT] = {};
        int offset = 0;
        while (offset < static_cast<int>(fields.length())) {
            int comma = fields.indexOf(',', offset);
            if (comma < 0) comma = fields.length();
            String token = fields.substring(offset, comma);
            token.trim();
            const int id = metricByKey(token);
            if (id < 0 || seen[id]) {
                lastError = "Invalid or duplicate selected telemetry field";
                return false;
            }
            seen[id] = true;
            ids[count++] = static_cast<uint8_t>(id);
            offset = comma + 1;
        }
        if (count == 0) { lastError = "Select at least one channel"; return false; }

        // NVS counter guarantees a new filename for every Start, even
        // after deleting recordings or rebooting the gateway.
        Preferences pref;
        if (!pref.begin("bike-logs", false)) {
            lastError = "Failed to open recording counter";
            return false;
        }
        uint32_t serial = pref.getUInt("next", 0);
        do { ++serial;
            char filename[32];
            snprintf(filename, sizeof(filename), "/log_%08lu.csv",
                     static_cast<unsigned long>(serial));
            path = filename;
        } while (serial != 0 && LittleFS.exists(path));
        if (serial == 0 || pref.putUInt("next", serial) != sizeof(uint32_t)) {
            pref.end();
            lastError = "Failed to reserve log file number";
            return false;
        }
        pref.end();

        file = LittleFS.open(path, "w");
        if (!file) { lastError = "Failed to create CSV file"; return false; }
        String header = "elapsed_ms,uptime_ms";
        for (uint8_t i = 0; i < count; ++i) {
            header += ',';
            header += METRICS[ids[i]].key;
        }
        header += "\n";
        if (file.print(header) != header.length()) {
            lastError = "Failed to write CSV header";
            stop();
            return false;
        }
        file.flush();
        active = true;
        startedMs = lastSampleMs = lastFlushMs = millis();
        rows = 0;
        lastError = "";
        return true;
    }

    void tick() {
        if (!active) return;
        const uint32_t now = millis();
        if (now - lastSampleMs < LOG_SAMPLE_MS) return;
        lastSampleMs = now;
        if (LittleFS.totalBytes() - LittleFS.usedBytes() < STORAGE_RESERVE_BYTES) {
            lastError = "Low flash storage: recording automatically closed";
            stop();
            return;
        }
        char line[1800];
        int used = snprintf(line, sizeof(line), "%lu,%lu",
            static_cast<unsigned long>(now - startedMs),
            static_cast<unsigned long>(now));
        if (used < 0 || used >= static_cast<int>(sizeof(line))) {
            lastError = "CSV line exceeded buffer"; stop(); return;
        }
        for (uint8_t i = 0; i < count; ++i) {
            Sample s = getMetric(static_cast<MetricId>(ids[i]));
            int n = s.valid
                ? snprintf(line + used, sizeof(line) - used, ",%.15g", s.value)
                : snprintf(line + used, sizeof(line) - used, ",");
            if (n < 0 || n >= static_cast<int>(sizeof(line)) - used) {
                lastError = "CSV line exceeded buffer"; stop(); return;
            }
            used += n;
        }
        line[used++] = '\n';
        if (file.write(reinterpret_cast<const uint8_t*>(line), used) != static_cast<size_t>(used)) {
            lastError = "Flash write failed: recording closed";
            stop();
            return;
        }
        ++rows;
        if (now - lastFlushMs >= LOG_FLUSH_MS) {
            file.flush();
            lastFlushMs = now;
        }
    }
};
CsvLogger logger;

// ============================================================
// HTTP API
// ============================================================
static void sendJSON(int status, const String& payload) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(status, "application/json", payload);
}
static void sendError(int status, const char* message) {
    String response = "{\"ok\":false,\"error\":\"";
    response += message;
    response += "\"}";
    sendJSON(status, response);
}
static void sendChannels() {
    String json;
    json.reserve(3800);
    json = "[";
    for (int i = 0; i < METRIC_COUNT; ++i) {
        if (i) json += ',';
        json += "{\"key\":\""; json += METRICS[i].key;
        json += "\",\"label\":\""; json += METRICS[i].label;
        json += "\",\"unit\":\""; json += METRICS[i].unit;
        json += "\",\"group\":\""; json += METRICS[i].group;
        json += "\",\"default\":";
        json += METRICS[i].selectedByDefault ? "true}" : "false}";
    }
    json += ']';
    sendJSON(200, json);
}
static String axisJson(const AxisTelemetry& a) {
    String j = "{";
    j += "\"position\":"; appendNumber(j, sample(fresh(a.lastEncoder), a.position));
    j += ",\"velocity\":"; appendNumber(j, sample(fresh(a.lastEncoder), a.velocity));
    j += ",\"iq\":"; appendNumber(j, sample(fresh(a.lastIq), a.iq));
    j += ",\"iqSet\":"; appendNumber(j, sample(fresh(a.lastIq), a.iqSet));
    j += ",\"state\":" + String(a.state);
    j += ",\"error\":" + String(a.axisError);
    j += ",\"motorFlags\":" + String(a.motorFlags);
    j += ",\"encoderFlags\":" + String(a.encoderFlags);
    j += ",\"controllerFlags\":" + String(a.controllerFlags);
    j += ",\"encoderFresh\":";
    j += fresh(a.lastEncoder) ? "true" : "false";
    j += ",\"iqFresh\":";
    j += fresh(a.lastIq) ? "true" : "false";
    j += ",\"heartbeatFresh\":";
    j += fresh(a.lastHeartbeat) ? "true" : "false";
    j += "}";
    return j;
}
static String bikeJson() {
    String j = "{";
    j += "\"connected\":";
    j += bike.connected() ? "true" : "false";
    // Retain previous BikeController JSON keys for compatibility.
    const char* names[] = {"cadence","roadSpeed","riderTorque","riderPower",
        "gearRatio","targetGearRatio","crankTorque","wheelTorque","batteryPower",
        "brakePower","powerLimitFactor","state","mode","batteryLimited","generatorLimited"};
    const MetricId ids[] = {B_CADENCE,B_ROAD,B_RIDER_TORQUE,B_RIDER_POWER,B_GEAR,
        B_TARGET_GEAR,B_CRANK_TORQUE,B_WHEEL_TORQUE,B_BATTERY_POWER,B_BRAKE_POWER,
        B_LIMIT_FACTOR,B_STATE,B_MODE,B_BATTERY_LIMITED,B_GENERATOR_LIMITED};
    for (size_t i = 0; i < sizeof(ids)/sizeof(ids[0]); ++i) {
        j += ",\""; j += names[i]; j += "\":";
        const Sample value = getMetric(ids[i]);
        if (ids[i] == B_BATTERY_LIMITED || ids[i] == B_GENERATOR_LIMITED) {
            j += !value.valid ? "null" : value.value != 0 ? "true" : "false";
        } else {
            appendNumber(j, value);
        }
    }
    j += ",\"protocolVersion\":" + String(bike.protocolVersion);
    j += ",\"frames\":" + String(bike.receivedFrames);
    j += ",\"fresh\":[";
    for (int i = 0; i < 6; ++i) {
        if (i) j += ',';
        j += bike.fresh(i) ? "true" : "false";
    }
    j += "]}";
    return j;
}
static void sendTelemetry() {
    String json;
    json.reserve(3500);
    json = "{\"uptimeMs\":" + String(millis());
    json += ",\"canFresh\":";
    json += fresh(lastAnyCan) ? "true" : "false";
    json += ",\"frames\":" + String(standardCanFrames);
    json += ",\"busVoltage\":"; appendNumber(json, getMetric(BUS_VOLTAGE));
    json += ",\"busCurrent\":"; appendNumber(json, getMetric(BUS_CURRENT));
    json += ",\"busFresh\":";
    json += fresh(lastBus) ? "true" : "false";
    json += ",\"crank\":" + axisJson(crank);
    json += ",\"wheel\":" + axisJson(wheel);
    json += ",\"bike\":" + bikeJson();
    json += ",\"signals\":{";
    for (int i = 0; i < METRIC_COUNT; ++i) {
        if (i) json += ',';
        json += '"'; json += METRICS[i].key; json += "\":";
        appendNumber(json, getMetric(static_cast<MetricId>(i)));
    }
    json += "}}";
    sendJSON(200, json);
}
static void sendStatus() {
    String j = "{\"ok\":true,\"storageReady\":";
    j += fsOkay ? "true" : "false";
    j += ",\"usedBytes\":" + String(fsOkay ? LittleFS.usedBytes() : 0);
    j += ",\"totalBytes\":" + String(fsOkay ? LittleFS.totalBytes() : 0);
    j += ",\"recording\":";
    j += logger.active ? "true" : "false";
    j += ",\"file\":\"" + logger.path + "\"";
    j += ",\"rows\":" + String(logger.rows);
    j += ",\"elapsedMs\":" + String(logger.active ? millis() - logger.startedMs : 0);
    j += ",\"lastError\":\"" + logger.lastError + "\"";
    j += ",\"fields\":[";
    for (uint8_t i = 0; i < logger.count; ++i) {
        if (i) j += ',';
        j += '"'; j += METRICS[logger.ids[i]].key; j += '"';
    }
    j += "]}";
    sendJSON(200, j);
}
// Accept only generated paths so the server cannot expose any other filesystem files.
static bool validLogPath(const String& name) {
    if (name.length() != 17 || !name.startsWith("/log_") || !name.endsWith(".csv")) return false;
    for (int i = 5; i < 13; ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
    }
    return true;
}
static void sendLogList() {
    if (!fsOkay) { sendError(503, "LittleFS unavailable"); return; }
    String json = "[";
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) { sendError(500, "Could not list logs"); return; }
    bool first = true;
    File f = root.openNextFile();
    while (f) {
        String name = f.name();
        if (!name.startsWith("/")) name = "/" + name;
        if (!f.isDirectory() && validLogPath(name)) {
            if (!first) json += ',';
            first = false;
            json += "{\"name\":\"" + name + "\",\"bytes\":" + String(f.size()) + "}";
        }
        f.close();
        f = root.openNextFile();
    }
    root.close();
    json += ']';
    sendJSON(200, json);
}
static void handleStartLog() {
    if (logger.active) { sendError(409, "Already recording"); return; }
    if (!server.hasArg("fields")) { sendError(400, "Missing fields"); return; }
    if (!logger.start(server.arg("fields"))) { sendError(400, logger.lastError.c_str()); return; }
    sendStatus();
}
static void handleStopLog() {
    if (!logger.active) { sendError(409, "Not recording"); return; }
    logger.stop();
    sendStatus();
}
static void handleDownload() {
    if (logger.active) { sendError(409, "Stop recording before downloading"); return; }
    if (!fsOkay) { sendError(503, "LittleFS unavailable"); return; }
    String name = server.arg("name");
    if (!validLogPath(name)) { sendError(400, "Invalid file name"); return; }
    File file = LittleFS.open(name, "r");
    if (!file) { sendError(404, "Log file not found"); return; }
    String shortName = name.substring(1);
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + shortName + "\"");
    server.sendHeader("Cache-Control", "no-store");
    server.streamFile(file, "text/csv");
    file.close();
}
static void handleDelete() {
    if (logger.active) { sendError(409, "Stop recording before deleting"); return; }
    if (!fsOkay) { sendError(503, "LittleFS unavailable"); return; }
    String name = server.arg("name");
    if (!validLogPath(name)) { sendError(400, "Invalid file name"); return; }
    if (!LittleFS.exists(name)) { sendError(404, "Log file not found"); return; }
    if (!LittleFS.remove(name)) { sendError(500, "Unable to delete log"); return; }
    sendJSON(200, "{\"ok\":true}");
}
static void serveHomepage() {
    // The page is embedded in firmware rather than the logging partition.
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("DigitalEBike gateway + logger");
    fsOkay = LittleFS.begin(false);  // Do NOT format automatically: preserve recordings.
    Serial.printf("LittleFS: %s\n", fsOkay ? "ready" : "not mounted");
    if (fsOkay) Serial.printf("Storage: %u / %u bytes\n",
        static_cast<unsigned>(LittleFS.usedBytes()),
        static_cast<unsigned>(LittleFS.totalBytes()));

    Serial.println(startCan() ? "CAN started" : "CAN failed");
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.print("WiFi address: "); Serial.println(WiFi.softAPIP());
    } else Serial.println("WiFi AP failed");

    server.on("/", HTTP_GET, serveHomepage);
    server.on("/api/channels", HTTP_GET, sendChannels);
    server.on("/api/telemetry", HTTP_GET, sendTelemetry);
    server.on("/api/log/status", HTTP_GET, sendStatus);
    server.on("/api/logs", HTTP_GET, sendLogList);
    server.on("/api/log/start", HTTP_POST, handleStartLog);
    server.on("/api/log/stop", HTTP_POST, handleStopLog);
    server.on("/api/log/download", HTTP_GET, handleDownload);
    server.on("/api/log/delete", HTTP_POST, handleDelete);
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
    Serial.println("Web server ready at http://192.168.4.1");
}

void loop() {
    twai_message_t message;
    for (int i = 0; i < 48; ++i) {
        if (twai_receive(&message, 0) != ESP_OK) break;
        processCan(message);
    }
    logger.tick();        // Recording continues when phone disconnects.
    server.handleClient();
    delay(1);
}
