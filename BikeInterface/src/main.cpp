
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println("DigitalEBike interface starting");
    Serial.println("M5Stack ATOMS3 Lite ready");
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint >= 1000)
    {
        lastPrint = millis();
        Serial.println("Bike interface running");
    }
}
