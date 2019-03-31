#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t TARGET_ID = 1;
const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

int dxl_goal_position[2];

void setup()
{
    Serial.begin(115200);

    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(TARGET_ID);

    delay(2000);

    dxl.torqueEnable(TARGET_ID, false);

    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID));

    dxl.minPositionLimit(TARGET_ID, 1400);
    dxl.verbose(TARGET_ID);
    dxl.maxPositionLimit(TARGET_ID, 1900);
    dxl.verbose(TARGET_ID);

    dxl_goal_position[0] = dxl.minPositionLimit(TARGET_ID);
    dxl_goal_position[1] = dxl.maxPositionLimit(TARGET_ID);

    dxl.velocityLimit(TARGET_ID, 30000);

    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID));

    dxl.torqueEnable(TARGET_ID, true);
}

void loop()
{
    static bool dir = true;

    dxl.goalPosition(TARGET_ID, dxl_goal_position[(size_t)dir]);
    dxl.verbose(TARGET_ID);

    delay(3000);

    Serial.print("current pos = ");
    Serial.println(dxl.presentPosition(TARGET_ID));
    dxl.verbose(TARGET_ID);

    dir = !dir;
}
