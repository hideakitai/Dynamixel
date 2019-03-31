
// Control table address
// Control table address is different in Dynamixel model
#define ADDR_PRO_INDIRECTADDRESS_BEGIN      168
#define ADDR_PRO_TORQUE_ENABLE                  562
#define ADDR_PRO_LED_RED                        563
#define ADDR_PRO_GOAL_POSITION                  596
#define ADDR_PRO_MOVING                         610
#define ADDR_PRO_PRESENT_POSITION               611
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634
#define ADDR_PRO_INDIRECTDATA_FOR_READ          639

// Data Byte Length
#define LEN_PRO_LED_RED                         1
#define LEN_PRO_GOAL_POSITION                   4
#define LEN_PRO_MOVING                          1
#define LEN_PRO_PRESENT_POSITION                4
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          5
#define LEN_PRO_INDIRECTDATA_FOR_READ           5

#define DXL_MINIMUM_POSITION_VALUE             -150000              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE              150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
// TODO: what's this??
#define DXL_MOVING_STATUS_THRESHOLD             20                  // Dynamixel moving status threshold


#include <ArduinoDynamixel.h>

const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

ArduinoDynamixel dx(PIN_RTS);

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial2.begin(DYNAMIXEL_BAUDRATE);
    dx.attach(Serial2, DYNAMIXEL_BAUDRATE);
    dx.addModel<arduino::Model::MX>

    delay(2000);

    // Indirect address would not accessible when the torque is already enabled
    dx.power(1, false);

// TODO:
#if 1
    dx.indirect.add(1, Reg::GOAL_POS); // automatically append
    dx.indirect.add(1, Reg::LED_R); // automatically append after
    dx.indirect.add(1, Reg::PRESENT_POS); // automatically append after
    dx.indirect.add(1, Reg::MOVING);
#else
    dx.setIndirectAddress(1, 0, ADDR_PRO_GOAL_POSITION + 0);
    dx.setIndirectAddress(1, 1, ADDR_PRO_GOAL_POSITION + 1);
    dx.setIndirectAddress(1, 2, ADDR_PRO_GOAL_POSITION + 2);
    dx.setIndirectAddress(1, 3, ADDR_PRO_GOAL_POSITION + 3);

    dx.setIndirectAddress(1, 4, ADDR_PRO_LED_RED);

    dx.setIndirectAddress(1, 5, ADDR_PRO_PRESENT_POSITION + 0);
    dx.setIndirectAddress(1, 6, ADDR_PRO_PRESENT_POSITION + 1);
    dx.setIndirectAddress(1, 7, ADDR_PRO_PRESENT_POSITION + 2);
    dx.setIndirectAddress(1, 8, ADDR_PRO_PRESENT_POSITION + 3);

    dx.setIndirectAddress(1, 9, ADDR_PRO_MOVING);
#endif

    dx.power(1, true);

#if 1
    // do not use sync_write/read
#else
    // TODO: do not use sync_write... maybe longer data cannot be read
    dx.set_address(ADDR_PRO_INDIRECTADDRESS_BEGIN + 0, LEN_PRO_INDIRECTDATA_FOR_WRITE);

    // TODO: sync read...
    dx.set_target(ADDR_PRO_INDIRECTADDRESS_BEGIN + 10, LEN_PRO_INDIRECTDATA_FOR_READ);
    dx.add_id(1);
#endif
}

void loop()
{
    static bool dir = false;

    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};
    uint8_t param_indirect_data_for_write[LEN_PRO_INDIRECTDATA_FOR_WRITE];
    uint8_t dxl_led_value[2] = {0x00, 0xFF};        // Dynamixel LED value
    param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[(size_t)dir]));
    param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[(size_t)dir]));
    param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[(size_t)dir]));
    param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[(size_t)dir]));
    param_indirect_data_for_write[4] = dxl_led_value[(size_t)dir];

    dx.add(1, param_indirect_data_for_write);

    dx.send();

    delay(500);

    dx.request();

    if (dx.available(1))
    {
        // TODO: sync_read cannot read more than 32bit data...
        // now, 5th byte is ignored...
        // in official example, first read 8bytes, after that, read 2byte...
        Serial.print("current pos = ");
        Serial.println(dx.data(1));

    }

    dir = !dir;
}