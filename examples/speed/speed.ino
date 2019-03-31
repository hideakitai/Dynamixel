#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want
const uint8_t TARGET_ID = 1;
const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS);

int goal_pos[2];

// TODO:
using namespace arduino;

void setup()
{
    Serial.begin(115200);

    delay(2000);

    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);
    dxl.addModel<DxlModel::X>(TARGET_ID);

    delay(2000);

    dxl.torqueEnable(TARGET_ID, false);

    Serial.print("op mode = ");
    Serial.println(dxl.operatingMode(TARGET_ID));
    Serial.print("drvie mode = ");
    Serial.println(dxl.driveMode(TARGET_ID));
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID));
    Serial.print("acc limit = ");
    Serial.println(dxl.accelerationLimit(TARGET_ID));
    Serial.print("vel limit = ");
    Serial.println(dxl.velocityLimit(TARGET_ID));
    Serial.print("vel goal = ");
    Serial.println(dxl.goalVelocity(TARGET_ID));
    Serial.print("profile velocity = ");
    Serial.println(dxl.profileVelocity(TARGET_ID));
    Serial.print("profile acceleration= ");
    Serial.println(dxl.profileAcceleration(TARGET_ID));


    dxl.minPositionLimit(TARGET_ID, 0);
    dxl.maxPositionLimit(TARGET_ID, 4095);
    dxl.profileVelocity(TARGET_ID, 171); // Max is 171, 0.229 * 170.305676856 = 39 [rpm] @12V
    dxl.profileAcceleration(TARGET_ID, 0); // 0: inifinity

    goal_pos[0] = dxl.minPositionLimit(TARGET_ID);
    goal_pos[1] = dxl.maxPositionLimit(TARGET_ID);

    Serial.print("op mode = ");
    Serial.println(dxl.operatingMode(TARGET_ID));
    Serial.print("drvie mode = ");
    Serial.println(dxl.driveMode(TARGET_ID));
    Serial.print("min pos = ");
    Serial.println(dxl.minPositionLimit(TARGET_ID));
    Serial.print("max pos = ");
    Serial.println(dxl.maxPositionLimit(TARGET_ID));
    Serial.print("acc limit = ");
    Serial.println(dxl.accelerationLimit(TARGET_ID));
    Serial.print("vel limit = ");
    Serial.println(dxl.velocityLimit(TARGET_ID));
    Serial.print("vel goal = ");
    Serial.println(dxl.goalVelocity(TARGET_ID));
    Serial.print("profile velocity = ");
    Serial.println(dxl.profileVelocity(TARGET_ID));
    Serial.print("profile acceleration= ");
    Serial.println(dxl.profileAcceleration(TARGET_ID));

    dxl.torqueEnable(TARGET_ID, true);
}

void loop()
{
    static bool dir = true;

#if 0
    dxl.goalPosition(TARGET_ID, goal_pos[(size_t)dir]);
    dxl.verbose(TARGET_ID);

    delay(3000);

    Serial.print("current pos = ");
    Serial.println(dxl.presentPosition(TARGET_ID));
    dxl.verbose(TARGET_ID);

    dir = !dir;
#else
    static uint16_t pos = 0;
    if (dir)
    {
        if (++pos > 4095) dir = !dir;
    }
    else
    {
        if (--pos < 1) dir = !dir;
    }
    dxl.goalPosition(TARGET_ID, pos);
    dxl.verbose(TARGET_ID);
    delay(20);
    Serial.println(pos);
#endif
}
