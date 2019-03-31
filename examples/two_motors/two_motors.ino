#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t TARGET_ID_1 = 1; // daisy-chained first dynamixel
const uint8_t TARGET_ID_2 = 2; // daisy-chained second dynamixel
const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

int dxl_goal_position_1[2];
int dxl_goal_position_2[2];

// TODO:
using namespace arduino;

void setup()
{
    Serial.begin(115200);

    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(TARGET_ID_1);
    dxl.addModel<DxlModel::MX>(TARGET_ID_2);

    delay(2000);

    dxl.torqueEnable(TARGET_ID_1, false);
    dxl.torqueEnable(TARGET_ID_2, false);

    Serial.println("ID 1 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    Serial.println("ID 2 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    dxl.minPositionLimit(TARGET_ID_1, 1400);
    dxl.verbose(TARGET_ID_1);
    dxl.maxPositionLimit(TARGET_ID_1, 1900);
    dxl.verbose(TARGET_ID_1);

    dxl.minPositionLimit(TARGET_ID_2, 1400);
    dxl.verbose(TARGET_ID_2);
    dxl.maxPositionLimit(TARGET_ID_2, 1900);
    dxl.verbose(TARGET_ID_2);

    dxl_goal_position_1[0] = dxl.minPositionLimit(TARGET_ID_1);
    dxl_goal_position_1[1] = dxl.maxPositionLimit(TARGET_ID_1);

    dxl_goal_position_2[0] = dxl.minPositionLimit(TARGET_ID_2);
    dxl_goal_position_2[1] = dxl.maxPositionLimit(TARGET_ID_2);

    dxl.velocityLimit(TARGET_ID_1, 30000);
    dxl.velocityLimit(TARGET_ID_2, 30000);

    Serial.println("ID 1 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    Serial.println("ID 2 : ");
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID_1));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID_1));

    dxl.torqueEnable(TARGET_ID_1, true);
    dxl.torqueEnable(TARGET_ID_2, true);
}

void loop()
{
    static bool dir = true;

    dxl.goalPosition(TARGET_ID_1, dxl_goal_position_1[(size_t)dir]);
    dxl.goalPosition(TARGET_ID_2, dxl_goal_position_2[(size_t)dir]);
    dxl.verbose(TARGET_ID_1);
    dxl.verbose(TARGET_ID_2);

    delay(3000);

    Serial.print("current pos (ID 1) = ");
    Serial.println(dxl.presentPosition(TARGET_ID_1));
    dxl.verbose(TARGET_ID_1);

    Serial.print("current pos (ID 2) = ");
    Serial.println(dxl.presentPosition(TARGET_ID_2));
    dxl.verbose(TARGET_ID_2);

    dir = !dir;
}
