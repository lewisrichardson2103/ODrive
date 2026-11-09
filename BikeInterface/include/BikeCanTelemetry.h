#pragma once

#include <Arduino.h>
#include <driver/twai.h>
#include <cmath>
#include <cstring>

class BikeCanTelemetry {
public:
    static constexpr uint32_t BASE_ID = 0x1ABCDE00;
    static constexpr uint32_t TIMEOUT_MS = 1000;
    float wheelCircumferenceM = 2.10f;  // Measure your actual tyre circumference.

    float cadenceRpm = 0.0f, wheelRadS = 0.0f;
    float riderTorqueNm = 0.0f, riderPowerW = 0.0f;
    float gearRatio = 0.0f, targetGearRatio = 0.0f;
    float crankTorqueNm = 0.0f, wheelTorqueNm = 0.0f;
    float batteryPowerW = 0.0f, brakePowerW = 0.0f;
    float powerLimitFactor = 1.0f;
    uint8_t state = 0, mode = 0, flags = 0, protocolVersion = 0;
    uint32_t lastReceived[6] = {};
    uint32_t receivedFrames = 0;

    bool process(const twai_message_t& msg) {
        if (!msg.extd || msg.rtr || msg.data_length_code != 8 ||
            msg.identifier < BASE_ID || msg.identifier >= BASE_ID + 6) return false;
        const uint8_t index = static_cast<uint8_t>(msg.identifier - BASE_ID);
        float a, b;
        std::memcpy(&a, msg.data, 4);
        std::memcpy(&b, msg.data + 4, 4);
        if (!std::isfinite(a) || (index != 5 && !std::isfinite(b))) return false;
        switch (index) {
            case 0: cadenceRpm = a; wheelRadS = b; break;
            case 1: riderTorqueNm = a; riderPowerW = b; break;
            case 2: gearRatio = a; targetGearRatio = b; break;
            case 3: crankTorqueNm = a; wheelTorqueNm = b; break;
            case 4: batteryPowerW = a; brakePowerW = b; break;
            case 5:
                if (msg.data[7] != 1) return false;
                powerLimitFactor = a;
                state = msg.data[4];
                mode = msg.data[5];
                flags = msg.data[6];
                protocolVersion = msg.data[7];
                break;
        }
        lastReceived[index] = millis();
        ++receivedFrames;
        return true;
    }

    bool fresh(uint8_t frame) const {
        return frame < 6 && lastReceived[frame] != 0 &&
               millis() - lastReceived[frame] < TIMEOUT_MS;
    }
    bool connected() const { return fresh(5); }
    bool batteryLimited() const { return (flags & 0x01) != 0; }
    bool generatorLimited() const { return (flags & 0x02) != 0; }
    float roadSpeedKmh() const {
        return wheelRadS * wheelCircumferenceM * (3.6f / 6.28318530718f);
    }
};
