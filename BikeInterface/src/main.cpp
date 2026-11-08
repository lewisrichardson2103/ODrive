
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <cstring>

// ======================================================
// Configuration
// ======================================================

constexpr gpio_num_t CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t CAN_RX = GPIO_NUM_6;

// Match these to the saved ODrive CAN configuration.
constexpr uint8_t CRANK_NODE_ID = 0;
constexpr uint8_t WHEEL_NODE_ID = 1;

// Change TWAI_TIMING_CONFIG_250KBITS below if needed.

constexpr char WIFI_SSID[] = "DigitalEBike";
constexpr char WIFI_PASSWORD[] = "DigitalBike123";

constexpr uint32_t DATA_TIMEOUT_MS = 2000;

// ODrive CAN Simple command IDs
constexpr uint8_t CMD_HEARTBEAT = 0x01;
constexpr uint8_t CMD_ENCODER = 0x09;
constexpr uint8_t CMD_IQ = 0x14;
constexpr uint8_t CMD_BUS = 0x17;

// ======================================================
// Telemetry
// ======================================================

struct AxisTelemetry
{
    float position = 0.0f;
    float velocity = 0.0f;   // Motor revolutions/s
    float iqSetpoint = 0.0f; // A
    float iqMeasured = 0.0f; // A

    uint32_t axisError = 0;
    uint8_t axisState = 0;

    uint32_t lastHeartbeat = 0;
    uint32_t lastEncoder = 0;
    uint32_t lastIq = 0;
};

struct BikeTelemetry
{
    AxisTelemetry crank;
    AxisTelemetry wheel;

    float busVoltage = 0.0f;
    float busCurrent = 0.0f;

    uint32_t frameCount = 0;
    uint32_t lastFrame = 0;
    uint32_t lastBus = 0;
};

BikeTelemetry telemetry;
WebServer server(80);

// ======================================================
// CAN decoding
// ======================================================

float readFloat32(const uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

uint32_t readUint32(const uint8_t *bytes)
{
    uint32_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

bool isFresh(uint32_t timestamp)
{
    return timestamp != 0 &&
           (millis() - timestamp < DATA_TIMEOUT_MS);
}

void processCanFrame(const twai_message_t &msg)
{
    // Only ODrive standard data frames.
    if (msg.extd || msg.rtr)
    {
        return;
    }

    telemetry.frameCount++;
    telemetry.lastFrame = millis();

    const uint8_t nodeId = msg.identifier >> 5;
    const uint8_t command = msg.identifier & 0x1F;

    AxisTelemetry *axis = nullptr;

    if (nodeId == CRANK_NODE_ID)
    {
        axis = &telemetry.crank;
    }
    else if (nodeId == WHEEL_NODE_ID)
    {
        axis = &telemetry.wheel;
    }
    else
    {
        return;
    }

    if (msg.data_length_code != 8)
    {
        return;
    }

    switch (command)
    {
    case CMD_HEARTBEAT:
        axis->axisError = readUint32(&msg.data[0]);
        axis->axisState = msg.data[4];
        axis->lastHeartbeat = millis();
        break;

    case CMD_ENCODER:
        axis->position = readFloat32(&msg.data[0]);
        axis->velocity = readFloat32(&msg.data[4]);
        axis->lastEncoder = millis();
        break;

    case CMD_IQ:
        axis->iqSetpoint = readFloat32(&msg.data[0]);
        axis->iqMeasured = readFloat32(&msg.data[4]);
        axis->lastIq = millis();
        break;

    case CMD_BUS:
        telemetry.busVoltage = readFloat32(&msg.data[0]);
        telemetry.busCurrent = readFloat32(&msg.data[4]);
        telemetry.lastBus = millis();
        break;
    }
}

bool startCan()
{
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX,
            CAN_RX,
            TWAI_MODE_NORMAL);

    general.rx_queue_len = 64;

    twai_timing_config_t timing =
        TWAI_TIMING_CONFIG_250KBITS();

    twai_filter_config_t filter =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(
            &general, &timing, &filter) != ESP_OK)
    {
        return false;
    }

    if (twai_start() != ESP_OK)
    {
        twai_driver_uninstall();
        return false;
    }

    return true;
}

// ======================================================
// Dashboard HTML
// ======================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport"
      content="width=device-width,initial-scale=1">

<title>DigitalEBike</title>

<style>
:root {
    color-scheme: dark;
    font-family: system-ui, sans-serif;
}
body {
    margin: 0;
    background: #10151d;
    color: #f0f4f8;
}
main {
    max-width: 900px;
    margin: auto;
    padding: 22px;
}
header {
    display: flex;
    justify-content: space-between;
    align-items: center;
}
h1 {
    margin: 0;
    font-size: 27px;
}
h2 {
    font-size: 17px;
    margin-top: 25px;
}
.subtitle {
    color: #9ca7b8;
    font-size: 13px;
}
.grid {
    display: grid;
    grid-template-columns: repeat(2,1fr);
    gap: 12px;
}
.card {
    background: #1c2633;
    border: 1px solid #303e4d;
    border-radius: 12px;
    padding: 18px;
}
.label {
    color: #aab7c5;
    font-size: 13px;
}
.value {
    font-size: clamp(25px,6vw,42px);
    font-weight: 650;
    margin-top: 12px;
    font-variant-numeric: tabular-nums;
}
.unit {
    font-size: 13px;
    color: #aab7c5;
}
.status {
    padding: 8px 12px;
    border-radius: 20px;
    background: #34404e;
    font-size: 12px;
}
.online { background: #155a40; }
.offline { background: #713030; }
.small {
    font-size: 13px;
    line-height: 1.9;
    color: #b7c3d1;
}
@media(max-width:430px) {
    main { padding: 13px; }
    .card { padding: 13px; }
    h1 { font-size: 23px; }
}
</style>
</head>

<body>
<main>
<header>
<div>
<h1>DigitalEBike</h1>
<div class="subtitle">Live drivetrain telemetry</div>
</div>
<div id="status" class="status">Connecting</div>
</header>

<h2>Motor speeds</h2>
<div class="grid">
<div class="card">
<div class="label">Crank motor</div>
<div class="value" id="crankSpeed">--</div>
<div class="unit">rev/s</div>
</div>
<div class="card">
<div class="label">Wheel motor</div>
<div class="value" id="wheelSpeed">--</div>
<div class="unit">rev/s</div>
</div>
</div>

<h2>Motor currents</h2>
<div class="grid">
<div class="card">
<div class="label">Crank measured Iq</div>
<div class="value" id="crankIq">--</div>
<div class="unit">A</div>
</div>
<div class="card">
<div class="label">Wheel measured Iq</div>
<div class="value" id="wheelIq">--</div>
<div class="unit">A</div>
</div>
<div class="card">
<div class="label">Crank requested Iq</div>
<div class="value" id="crankIqSet">--</div>
<div class="unit">A</div>
</div>
<div class="card">
<div class="label">Wheel requested Iq</div>
<div class="value" id="wheelIqSet">--</div>
<div class="unit">A</div>
</div>
</div>

<h2>Power supply</h2>
<div class="grid">
<div class="card">
<div class="label">DC bus voltage</div>
<div class="value" id="voltage">--</div>
<div class="unit">V</div>
</div>
<div class="card">
<div class="label">DC bus current</div>
<div class="value" id="current">--</div>
<div class="unit">A</div>
</div>
</div>

<h2>Diagnostics</h2>
<div class="card small">
<div>Crank state: <span id="crankState">--</span></div>
<div>Wheel state: <span id="wheelState">--</span></div>
<div>Crank error: <span id="crankError">--</span></div>
<div>Wheel error: <span id="wheelError">--</span></div>
<div>CAN frames received: <span id="frames">0</span></div>
</div>
</main>

<script>
const el = id => document.getElementById(id);

function number(value, valid, digits = 1) {
    return valid ? Number(value).toFixed(digits) : "--";
}

async function update() {
    try {
        const response = await fetch('/api/telemetry',
            {cache:'no-store'});

        if (!response.ok) throw Error("HTTP error");

        const d = await response.json();

        el('crankSpeed').textContent =
            number(d.crank.velocity, d.crank.encoderFresh, 2);
        el('wheelSpeed').textContent =
            number(d.wheel.velocity, d.wheel.encoderFresh, 2);

        el('crankIq').textContent =
            number(d.crank.iq, d.crank.iqFresh);
        el('wheelIq').textContent =
            number(d.wheel.iq, d.wheel.iqFresh);

        el('crankIqSet').textContent =
            number(d.crank.iqSet, d.crank.iqFresh);
        el('wheelIqSet').textContent =
            number(d.wheel.iqSet, d.wheel.iqFresh);

        el('voltage').textContent =
            number(d.busVoltage, d.busFresh);
        el('current').textContent =
            number(d.busCurrent, d.busFresh);

        el('crankState').textContent =
            d.crank.heartbeatFresh ? d.crank.state : '--';
        el('wheelState').textContent =
            d.wheel.heartbeatFresh ? d.wheel.state : '--';

        el('crankError').textContent =
            d.crank.heartbeatFresh ? d.crank.error : '--';
        el('wheelError').textContent =
            d.wheel.heartbeatFresh ? d.wheel.error : '--';

        el('frames').textContent = d.frames;

        const status = el('status');
        status.textContent = d.canFresh ? 'CAN connected' :
                                         'CAN offline';
        status.className = 'status ' +
            (d.canFresh ? 'online' : 'offline');

    } catch(e) {
        const status = el('status');
        status.textContent = 'Gateway offline';
        status.className = 'status offline';
    }
}

update();
setInterval(update, 200);
</script>
</body>
</html>
)HTML";

// ======================================================
// JSON API
// ======================================================

String axisJson(const AxisTelemetry &a)
{
    String s = "{";

    s += "\"velocity\":" + String(a.velocity, 4);
    s += ",\"iq\":" + String(a.iqMeasured, 3);
    s += ",\"iqSet\":" + String(a.iqSetpoint, 3);
    s += ",\"state\":" + String(a.axisState);
    s += ",\"error\":" + String(a.axisError);

    s += ",\"encoderFresh\":";
    s += isFresh(a.lastEncoder) ? "true" : "false";

    s += ",\"iqFresh\":";
    s += isFresh(a.lastIq) ? "true" : "false";

    s += ",\"heartbeatFresh\":";
    s += isFresh(a.lastHeartbeat) ? "true" : "false";

    s += "}";
    return s;
}

void sendTelemetry()
{
    String json;
    json.reserve(650);

    json = "{";
    json += "\"crank\":" + axisJson(telemetry.crank);
    json += ",\"wheel\":" + axisJson(telemetry.wheel);

    json += ",\"busVoltage\":" + String(telemetry.busVoltage, 2);
    json += ",\"busCurrent\":" + String(telemetry.busCurrent, 2);

    json += ",\"busFresh\":";
    json += isFresh(telemetry.lastBus) ? "true" : "false";

    json += ",\"canFresh\":";
    json += isFresh(telemetry.lastFrame) ? "true" : "false";

    json += ",\"frames\":" + String(telemetry.frameCount);

    json += "}";

    server.send(200, "application/json", json);
}

// ======================================================
// Setup / Loop
// ======================================================

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println("DigitalEBike CAN Gateway");

    if (!startCan())
    {
        Serial.println("CAN initialisation FAILED");
    }
    else
    {
        Serial.println("CAN receiver started");
    }

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD))
    {
        Serial.println("WiFi AP failed");
    }
    else
    {
        Serial.print("WiFi IP: ");
        Serial.println(WiFi.softAPIP());
    }

    server.on("/", HTTP_GET, []()
              { server.send_P(200, "text/html", INDEX_HTML); });

    server.on("/api/telemetry", HTTP_GET, sendTelemetry);

    server.begin();
    Serial.println("Web server ready");
}

void loop()
{
    twai_message_t msg;

    // Drain available CAN messages without blocking
    // the HTTP server for long periods.
    for (int i = 0; i < 32; i++)
    {
        if (twai_receive(&msg, 0) != ESP_OK)
        {
            break;
        }
        processCanFrame(msg);
    }

    server.handleClient();
    delay(1);
}
