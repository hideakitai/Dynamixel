#include <ArduinoDynamixel.h>

const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

ArduinoDynamixel dx(PIN_RTS);


// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

#define DXL_MINIMUM_POSITION_VALUE     -150000              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

void setup()
{
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  //bool dxl_getdata_result = false;                 // GetParam result
  int32_t dxl1_present_position = 0, dxl2_present_position = 0;              // Present position

    Serial.begin(115200);

    delay(2000);

    Serial2.begin(DYNAMIXEL_BAUDRATE);
    dx.attach(Serial2, DYNAMIXEL_BAUDRATE);

    delay(2000);

    dx.power(1, true);

    dx.set_target(ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    // TODO: varidic arguments...
    dx.add_id(1);
    dx.add_id(2);
}

void loop()
{
    static bool dir = true;

    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};
    uint8_t param_goal_position[4];

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[(size_t)dir]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[(size_t)dir]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[(size_t)dir]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[(size_t)dir]));

    dx.set_address(ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
    dx.add(1, param_goal_position);
    dx.add(2, param_goal_position);

    dx.send();

    delay(500);

    dx.request();

    if (dx.availablel(1))
    {
        Serial.print("current pos (ID 1) = "):
        Serial.println(dx.data(1));
    }
    if (dx.availablel(2))
    {
        Serial.print("current pos (ID 2) = "):
        Serial.println(dx.data(2));
    }

    dir = !dir;
}
