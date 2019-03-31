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
    Serial.println("start reboot... see LED flickering");
    dxl.reboot(TARGET_ID);
    delay(5000);
}
