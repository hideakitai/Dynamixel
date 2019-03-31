#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t TARGET_ID = 1;
const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

void setup()
{
    Serial.begin(115200);

    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(TARGET_ID);

    delay(2000);
}

void loop()
{
    if (dxl.ping(TARGET_ID))
    {
        Serial.print("[ID:"); Serial.print(TARGET_ID);
        Serial.print("] ping Succeeded. Dynamixel model number : ");
        Serial.println(dxl.modelNumber(TARGET_ID), HEX);
    }
    else
    {
        Serial.print("[ID:"); Serial.print(TARGET_ID);
        Serial.print("] ping Failed.");
        dxl.verbose(TARGET_ID);
    }
    delay(1000);
}
