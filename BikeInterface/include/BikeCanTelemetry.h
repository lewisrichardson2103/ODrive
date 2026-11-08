
#pragma once

#include <Arduino.h>
#include <driver/twai.h>
#include <cstring>
#include <cmath>

class BikeCanTelemetry
{
public:
    static constexpr uint32_t BASE_ID = 0x1ABCDE00;
    static constexpr uint32_t TIMEOUT_MS = 1000;

    // Change to your measured wheel circumference.
    float wheelCircumferenceM = 2.10f;

    // Bicycle measurements
    float cadenceRpm = 0.0f;
    float wheelRadS = 0.0f;

    float riderTorqueNm = 0.0f;
    float riderPowerW = 0.0f;

    float gearRatio = 0.0f;
    float targetGearRatio = 0.0f;

    float crankTorqueNm = 0.0f;
    float wheelTorqueNm = 0.0f;

    float batteryPowerW = 0.0f;
    float brakePowerW = 0.0f;

    float powerLimitFactor = 1.0f;

    uint8_t state = 0;
    uint8_t mode = 0;
    uint8_t flags = 0;
    uint8_t protocolVersion = 0;

    uint32_t lastReceived[6] = {};
    uint32_t receivedFrames = 0;

    bool process(const twai_message_t &msg)
    {
        if (!msg.extd || msg.rtr ||
            msg.data_length_code != 8)
        {
            return false;
        }

        if (msg.identifier < BASE_ID ||
            msg.identifier >= BASE_ID + 6)
        {
            return false;
        }

        const uint8_t frame =
            static_cast<uint8_t>(
                msg.identifier - BASE_ID);

        const float v0 = readFloat(&msg.data[0]);
        const float v1 = readFloat(&msg.data[4]);

        if (!std::isfinite(v0))
        {
            return false;
        }

        if (frame != 5 && !std::isfinite(v1))
        {
            return false;
        }

        switch (frame)
        {
        case 0:
            cadenceRpm = v0;
            wheelRadS = v1;
            break;

        case 1:
            riderTorqueNm = v0;
            riderPowerW = v1;
            break;

        case 2:
            gearRatio = v0;
            targetGearRatio = v1;
            break;

        case 3:
            crankTorqueNm = v0;
            wheelTorqueNm = v1;
            break;

        case 4:
            batteryPowerW = v0;
            brakePowerW = v1;
            break;

        case 5:
            if (msg.data[7] != 1)
            {
                return false;
            }

            powerLimitFactor = v0;
            state = msg.data[4];
            mode = msg.data[5];
            flags = msg.data[6];
            protocolVersion = msg.data[7];
            break;
        }

        lastReceived[frame] = millis();
        ++receivedFrames;

        return true;
    }

    bool fresh(uint8_t frame) const
    {
        return frame < 6 &&
               lastReceived[frame] != 0 &&
               millis() - lastReceived[frame] < TIMEOUT_MS;
    }

    bool connected() const
    {
        return fresh(5);
    }

    float roadSpeedKmh() const
    {
        // radians/s -> rev/s -> m/s -> km/h
        return wheelRadS *
               wheelCircumferenceM *
               (3.6f / 6.28318530718f);
    }

    bool batteryLimited() const
    {
        return (flags & 0x01) != 0;
    }

    bool generatorLimited() const
    {
        return (flags & 0x02) != 0;
    }

private:
    static float readFloat(const uint8_t *data)
    {
        float result;
        std::memcpy(&result, data, sizeof(result));
        return result;
    }
};
