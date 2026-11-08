
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <cstring>
#include "BikeCanTelemetry.h"

BikeCanTelemetry bikeTelemetry;

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
    // Handle bicycle-specific extended CAN messages.
    if (msg.extd)
    {
        bikeTelemetry.process(msg);
        return;
    }

    // Existing ODrive CAN Simple decoding follows.
    if (msg.rtr)
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

<title>DigitalEBike Dashboard</title>

<style>
:root {
    color-scheme: dark;
    font-family: system-ui, sans-serif;
    --bg: #10151d;
    --card: #1c2633;
    --border: #303e4d;
    --text: #f0f4f8;
    --muted: #9ca7b8;
    --accent: #27c9ae;
}

* { box-sizing: border-box; }

body {
    margin: 0;
    background: var(--bg);
    color: var(--text);
}

main {
    max-width: 1100px;
    margin: auto;
    padding: 20px;
}

header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
    margin-bottom: 25px;
}

h1 {
    margin: 0;
    font-size: 26px;
}

h2 {
    font-size: 18px;
    margin: 28px 0 12px;
    padding-bottom: 8px;
    border-bottom: 1px solid var(--border);
}

.subtitle {
    color: var(--muted);
    font-size: 13px;
    margin-top: 4px;
}

.grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 12px;
}

.card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 16px;
    min-width: 0;
}

.label {
    color: var(--muted);
    font-size: 12px;
}

.value {
    font-size: clamp(22px, 3vw, 36px);
    font-weight: 650;
    margin-top: 12px;
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
}

.unit {
    color: var(--muted);
    font-size: 12px;
    margin-top: 3px;
}

.status {
    padding: 8px 12px;
    border-radius: 20px;
    background: #34404e;
    font-size: 12px;
    white-space: nowrap;
}

.online { background: #155a40; }
.offline { background: #713030; }

.diagnostics {
    font-size: 13px;
    line-height: 2.2;
}

.diagnostic-row {
    display: flex;
    justify-content: space-between;
    gap: 12px;
    border-bottom: 1px solid var(--border);
}

.diagnostic-row:last-child {
    border-bottom: none;
}

.diagnostic-value {
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    text-align: right;
}

.limit-active { color: #ffad66; }
.limit-ok { color: var(--accent); }

@media(max-width:750px) {
    .grid { grid-template-columns: repeat(2,1fr); }
}

@media(max-width:430px) {
    main { padding: 12px; }
    .card { padding: 13px; }
    h1 { font-size: 21px; }
    .value { font-size: 27px; }
}
</style>
</head>

<body>
<main>

<header>
    <div>
        <h1>DigitalEBike</h1>
        <div class="subtitle">
            Live drivetrain telemetry
        </div>
    </div>
    <div id="status" class="status">Connecting</div>
</header>

<!-- ========================== -->
<!-- Bicycle -->
<!-- ========================== -->

<h2>Bicycle</h2>
<div class="grid">
    <div class="card">
        <div class="label">Road speed</div>
        <div class="value" id="roadSpeed">--</div>
        <div class="unit">km/h</div>
    </div>
    <div class="card">
        <div class="label">Cadence</div>
        <div class="value" id="cadence">--</div>
        <div class="unit">rpm</div>
    </div>
</div>

<!-- ========================== -->
<!-- Rider -->
<!-- ========================== -->

<h2>Rider input</h2>
<div class="grid">
    <div class="card">
        <div class="label">Rider torque</div>
        <div class="value" id="riderTorque">--</div>
        <div class="unit">Nm</div>
    </div>
    <div class="card">
        <div class="label">Rider power</div>
        <div class="value" id="riderPower">--</div>
        <div class="unit">W</div>
    </div>
</div>

<!-- ========================== -->
<!-- Virtual drivetrain -->
<!-- ========================== -->

<h2>Virtual drivetrain</h2>
<div class="grid">
    <div class="card">
        <div class="label">Active gear ratio</div>
        <div class="value" id="gearRatio">--</div>
        <div class="unit">ratio</div>
    </div>
    <div class="card">
        <div class="label">Target gear ratio</div>
        <div class="value" id="targetGearRatio">--</div>
        <div class="unit">ratio</div>
    </div>
    <div class="card">
        <div class="label">Crank command torque</div>
        <div class="value" id="crankTorque">--</div>
        <div class="unit">Nm</div>
    </div>
    <div class="card">
        <div class="label">Wheel command torque</div>
        <div class="value" id="wheelTorque">--</div>
        <div class="unit">Nm</div>
    </div>
</div>

<!-- ========================== -->
<!-- Power -->
<!-- ========================== -->

<h2>Power management</h2>
<div class="grid">
    <div class="card">
        <div class="label">Battery power</div>
        <div class="value" id="batteryPower">--</div>
        <div class="unit">W (+ discharge)</div>
    </div>
    <div class="card">
        <div class="label">Brake resistor power</div>
        <div class="value" id="brakePower">--</div>
        <div class="unit">W</div>
    </div>
    <div class="card">
        <div class="label">Power limit factor</div>
        <div class="value" id="powerLimit">--</div>
        <div class="unit">%</div>
    </div>
</div>

<div class="card diagnostics" style="margin-top:12px">
    <div class="diagnostic-row">
        <span>Battery discharge limiting</span>
        <span class="diagnostic-value"
              id="batteryLimited">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Generator power limiting</span>
        <span class="diagnostic-value"
              id="generatorLimited">--</span>
    </div>
</div>

<!-- ========================== -->
<!-- Crank motor -->
<!-- ========================== -->

<h2>Crank motor</h2>
<div class="grid">
    <div class="card">
        <div class="label">Motor velocity</div>
        <div class="value" id="crankSpeed">--</div>
        <div class="unit">rev/s</div>
    </div>
    <div class="card">
        <div class="label">Measured Iq</div>
        <div class="value" id="crankIq">--</div>
        <div class="unit">A</div>
    </div>
    <div class="card">
        <div class="label">Requested Iq</div>
        <div class="value" id="crankIqSet">--</div>
        <div class="unit">A</div>
    </div>
</div>

<!-- ========================== -->
<!-- Wheel motor -->
<!-- ========================== -->

<h2>Wheel motor</h2>
<div class="grid">
    <div class="card">
        <div class="label">Motor velocity</div>
        <div class="value" id="wheelSpeed">--</div>
        <div class="unit">rev/s</div>
    </div>
    <div class="card">
        <div class="label">Measured Iq</div>
        <div class="value" id="wheelIq">--</div>
        <div class="unit">A</div>
    </div>
    <div class="card">
        <div class="label">Requested Iq</div>
        <div class="value" id="wheelIqSet">--</div>
        <div class="unit">A</div>
    </div>
</div>

<!-- ========================== -->
<!-- DC bus -->
<!-- ========================== -->

<h2>DC bus</h2>
<div class="grid">
    <div class="card">
        <div class="label">Bus voltage</div>
        <div class="value" id="voltage">--</div>
        <div class="unit">V</div>
    </div>
    <div class="card">
        <div class="label">Bus current</div>
        <div class="value" id="current">--</div>
        <div class="unit">A</div>
    </div>
</div>

<!-- ========================== -->
<!-- Diagnostics -->
<!-- ========================== -->

<h2>Controller diagnostics</h2>
<div class="card diagnostics">
    <div class="diagnostic-row">
        <span>BikeController state</span>
        <span class="diagnostic-value"
              id="bikeState">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Operating mode</span>
        <span class="diagnostic-value"
              id="bikeMode">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Crank axis state</span>
        <span class="diagnostic-value"
              id="crankState">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Wheel axis state</span>
        <span class="diagnostic-value"
              id="wheelState">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Crank axis error</span>
        <span class="diagnostic-value"
              id="crankError">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Wheel axis error</span>
        <span class="diagnostic-value"
              id="wheelError">--</span>
    </div>
</div>

<h2>CAN diagnostics</h2>
<div class="card diagnostics">
    <div class="diagnostic-row">
        <span>Standard CAN frames</span>
        <span class="diagnostic-value"
              id="frames">0</span>
    </div>
    <div class="diagnostic-row">
        <span>Bike telemetry frames</span>
        <span class="diagnostic-value"
              id="bikeFrames">0</span>
    </div>
    <div class="diagnostic-row">
        <span>Bike telemetry protocol</span>
        <span class="diagnostic-value"
              id="protocolVersion">--</span>
    </div>
    <div class="diagnostic-row">
        <span>Telemetry frame freshness</span>
        <span class="diagnostic-value"
              id="frameFreshness">--</span>
    </div>
</div>

</main>

<script>
const el = id => document.getElementById(id);

const setText = (id, value) => {
    el(id).textContent = value;
};

function formatNumber(value, valid, digits = 1) {
    if (!valid || !Number.isFinite(Number(value))) {
        return '--';
    }
    return Number(value).toFixed(digits);
}

function setNumber(id, value, valid, digits = 1) {
    setText(id, formatNumber(value, valid, digits));
}

function setFlag(id, value, valid) {
    const element = el(id);

    if (!valid) {
        element.textContent = '--';
        element.className = 'diagnostic-value';
        return;
    }

    element.textContent = value ? 'ACTIVE' : 'Inactive';
    element.className = 'diagnostic-value ' +
        (value ? 'limit-active' : 'limit-ok');
}

function updateBike(b) {
    const fresh = b.fresh || [];

    setNumber('roadSpeed', b.roadSpeed, fresh[0], 1);
    setNumber('cadence', b.cadence, fresh[0], 1);

    setNumber('riderTorque', b.riderTorque, fresh[1], 2);
    setNumber('riderPower', b.riderPower, fresh[1], 1);

    setNumber('gearRatio', b.gearRatio, fresh[2], 3);
    setNumber('targetGearRatio',
              b.targetGearRatio, fresh[2], 3);

    setNumber('crankTorque', b.crankTorque, fresh[3], 2);
    setNumber('wheelTorque', b.wheelTorque, fresh[3], 2);

    setNumber('batteryPower', b.batteryPower, fresh[4], 1);
    setNumber('brakePower', b.brakePower, fresh[4], 1);

    setNumber('powerLimit',
              b.powerLimitFactor * 100, fresh[5], 1);

    setFlag('batteryLimited',
            b.batteryLimited, fresh[5]);

    setFlag('generatorLimited',
            b.generatorLimited, fresh[5]);

    setText('bikeState', fresh[5] ? b.state : '--');
    setText('bikeMode', fresh[5] ? b.mode : '--');

    setText('bikeFrames', b.frames);
    setText('protocolVersion', fresh[5] ? 1 : '--');

    setText('frameFreshness',
        fresh.map(f => f ? '●' : '○').join(' '));
}

function updateAxis(prefix, axis) {
    setNumber(prefix + 'Speed',
              axis.velocity, axis.encoderFresh, 2);

    setNumber(prefix + 'Iq',
              axis.iq, axis.iqFresh, 2);

    setNumber(prefix + 'IqSet',
              axis.iqSet, axis.iqFresh, 2);

    setText(prefix + 'State',
            axis.heartbeatFresh ? axis.state : '--');

    setText(prefix + 'Error',
            axis.heartbeatFresh ?
                '0x' + Number(axis.error).toString(16).toUpperCase()
                : '--');
}

async function update() {
    try {
        const response = await fetch('/api/telemetry', {
            cache: 'no-store'
        });

        if (!response.ok) {
            throw Error('HTTP error');
        }

        const d = await response.json();

        if (d.bike) {
            updateBike(d.bike);
        }

        updateAxis('crank', d.crank);
        updateAxis('wheel', d.wheel);

        setNumber('voltage',
                  d.busVoltage, d.busFresh, 2);

        setNumber('current',
                  d.busCurrent, d.busFresh, 2);

        setText('frames', d.frames);

        const status = el('status');

        if (d.bike && d.bike.connected) {
            status.textContent = 'Bike connected';
            status.className = 'status online';
        } else if (d.canFresh) {
            status.textContent = 'ODrive CAN connected';
            status.className = 'status online';
        } else {
            status.textContent = 'CAN offline';
            status.className = 'status offline';
        }

    } catch(error) {
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

String bikeTelemetryJson()
{
    const auto &b = bikeTelemetry;

    String json;
    json.reserve(700);

    json = "{";

    json += "\"connected\":";
    json += b.connected() ? "true" : "false";

    json += ",\"cadence\":" +
            String(b.cadenceRpm, 2);

    json += ",\"roadSpeed\":" +
            String(b.roadSpeedKmh(), 2);

    json += ",\"riderTorque\":" +
            String(b.riderTorqueNm, 2);

    json += ",\"riderPower\":" +
            String(b.riderPowerW, 2);

    json += ",\"gearRatio\":" +
            String(b.gearRatio, 3);

    json += ",\"targetGearRatio\":" +
            String(b.targetGearRatio, 3);

    json += ",\"crankTorque\":" +
            String(b.crankTorqueNm, 2);

    json += ",\"wheelTorque\":" +
            String(b.wheelTorqueNm, 2);

    json += ",\"batteryPower\":" +
            String(b.batteryPowerW, 2);

    json += ",\"brakePower\":" +
            String(b.brakePowerW, 2);

    json += ",\"powerLimitFactor\":" +
            String(b.powerLimitFactor, 3);

    json += ",\"state\":" +
            String(b.state);

    json += ",\"mode\":" +
            String(b.mode);

    json += ",\"batteryLimited\":";
    json += b.batteryLimited() ? "true" : "false";

    json += ",\"generatorLimited\":";
    json += b.generatorLimited() ? "true" : "false";

    json += ",\"frames\":" +
            String(b.receivedFrames);

    // Each telemetry frame has an independent
    // freshness indicator.
    json += ",\"fresh\":[";
    for (uint8_t i = 0; i < 6; ++i)
    {
        if (i)
            json += ",";
        json += b.fresh(i) ? "true" : "false";
    }
    json += "]";

    json += "}";

    return json;
}

void sendTelemetry()
{
    String json;
    json.reserve(1600);

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

    json += ",\"bike\":" + bikeTelemetryJson();

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
