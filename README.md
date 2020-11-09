# Dynamixel

Arduino library for [Dynamixel](http://en.robotis.com/model/page.php?co_id=prd_dynamixel_x).

## Feature

This library is a wrapper of [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for Arduino.
It makes to use Dynamixel easier.

- Pro, X, MX, and others(DX, RX, AX) series are supported
- Both Protocol 1.0 and 2.0 are supported (X, MX series supports only 2.0)
- Almost basic APIs are wrapped and simplified.
- This library can control multiple motors which is daisy-chained in one RS485 bus.
- By default, it is assumed that one RTS pin is used.
- You can also use two pins to control enable/disable RX/TX respectively.

Please feel free to report issues and PRs.

### Supported & Tested Platforms

This library supports almost all Arduino boards.


### Supported Models

- Pro Series
- X Series
- MX Series
- Others (DX, RX, AX)

### API Limitations

- REG_WRITE/ACTION is not wrapped
- SYNC READ/WRITE, BULK READ/WRITE is not wrapped
- INDIRECT ACCESS is not wrapped

## Usage

``` C++
#include <Dynamixel.h>

#define DYNAMIXEL_SERIAL Serial2 // change as you want

const uint8_t TARGET_ID = 1;
const uint8_t PIN_RTS = 11;
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

Dynamixel dxl(PIN_RTS); // create instance with RTS pin

int dxl_goal_position[2];

void setup()
{
    // initialize Serial for RS485 and attach it to Dynamixel libaray
    DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
    dxl.attach(DYNAMIXEL_SERIAL, DYNAMIXEL_BAUDRATE);

    // add model with id
    dxl.addModel<DxlModel::X>(TARGET_ID);
    // dxl.addModel<DxlModel::PRO>(TARGET_ID);
    // dxl.addModel<DxlModel::MX>(TARGET_ID);
    // dxl.addModel<DxlModel::OTHER>(TARGET_ID);

    delay(2000);

    dxl.torqueEnable(TARGET_ID, false);

    dxl.minPositionLimit(TARGET_ID, 1400);
    dxl.maxPositionLimit(TARGET_ID, 1900);
    dxl_goal_position[0] = dxl.minPositionLimit(TARGET_ID); // get min pos limit
    dxl_goal_position[1] = dxl.maxPositionLimit(TARGET_ID); // get max pos limit
    dxl.velocityLimit(TARGET_ID, 30000);

    dxl.torqueEnable(TARGET_ID, true);
}

bool dir = true; // CW or CCW

void loop()
{
    dxl.goalPosition(TARGET_ID, dxl_goal_position[(size_t)dir]); // move to position

    delay(3000);

    Serial.print("current pos = ");
    Serial.println(dxl.presentPosition(TARGET_ID)); // get current position

    dir = !dir; // reverse direction
}
```

For more examples, see `examples` folder.

## APIs

``` C++
// initialize
template <Model> void addModel(uint8_t id)
void attach(Stream& s, size_t baud)
uint8_t size() const

// wrapper for instructions
bool ping(uint8_t id)
Models ping() // broadcast
bool factoryReset(uint8_t id, ResetMode mode = ResetMode::EXC_ID_BAUD)
bool factoryReset(ResetMode mode = ResetMode::EXC_ID_BAUD)
bool reboot(uint8_t id)
void verbose(uint8_t id)
bool write(uint8_t id, Reg reg, uint32_t data)
uint32_t read(uint8_t id, Reg reg)

// wrappers for control table
// read values
uint16_t modelNumber(uint8_t id)
uint32_t modelInformation(uint8_t id)
uint8_t versionOfFirmware(uint8_t id)
uint8_t id(uint8_t id)
uint8_t baudrate(uint8_t id)
uint8_t returnDelayTime(uint8_t id)
uint8_t driveMode(uint8_t id)
uint8_t operatingMode(uint8_t id)
uint8_t secondaryId(uint8_t id)
uint8_t protocolVersion(uint8_t id)
int32_t homingOffset(uint8_t id)
uint32_t movingThreshold(uint8_t id)
uint8_t temperatureLimit(uint8_t id)
uint16_t maxVoltageLimit(uint8_t id)
uint16_t minVoltageLimit(uint8_t id)
uint16_t pwmLimit(uint8_t id)
uint16_t currentLimit(uint8_t id)
uint32_t accelerationLimit(uint8_t id)
uint32_t velocityLimit(uint8_t id)
uint32_t maxPositionLimit(uint8_t id)
uint32_t minPositionLimit(uint8_t id)
uint8_t shutdown(uint8_t id)
bool torqueEnable(uint8_t id)
uint8_t led(uint8_t id)
uint8_t statusReturnLevel(uint8_t id)
uint8_t registerdInstruction(uint8_t id)
uint8_t hardwareErrorStatus(uint8_t id)
uint16_t velocityIGain(uint8_t id)
uint16_t velocityPGain(uint8_t id)
uint16_t positionDGain(uint8_t id)
uint16_t positionIGain(uint8_t id)
uint16_t positionPGain(uint8_t id)
uint16_t feedForwardAccelerationGain(uint8_t id)
uint16_t feedForwardVelocityGain(uint8_t id)
int8_t busWatchdog(uint8_t id)
int16_t goalPwm(uint8_t id)
int16_t goalCurrent(uint8_t id)
int32_t goalVelocity(uint8_t id)
uint32_t profileAcceleration(uint8_t id)
uint32_t profileVelocity(uint8_t id)
int32_t goalPosition(uint8_t id)
uint16_t realTimeTick(uint8_t id)
uint8_t moving(uint8_t id)
uint8_t movingStatus(uint8_t id)
int16_t presentPwm(uint8_t id)
int16_t presentCurrent(uint8_t id)
int32_t presentVelocity(uint8_t id)
int32_t presentPosition(uint8_t id)
uint32_t velocityTrajectory(uint8_t id)
uint32_t positionTrajectory(uint8_t id)
uint16_t presentInputVoltage(uint8_t id)
uint8_t presentTemperature(uint8_t id)

// write values
bool id(uint8_t id, uint8_t x)
bool baudrate(uint8_t id, uint8_t x)
bool returnDelayTime(uint8_t id, uint8_t x)
bool driveMode(uint8_t id, uint8_t x)
bool operatingMode(uint8_t id, uint8_t x)
bool secondaryId(uint8_t id, uint8_t x)
bool protocolVersion(uint8_t id, uint8_t x)
bool homingOffset(uint8_t id, int32_t x)
bool movingThreshold(uint8_t id, uint32_t x)
bool temperatureLimit(uint8_t id, uint8_t x)
bool maxVoltageLimit(uint8_t id, uint16_t x)
bool minVoltageLimit(uint8_t id, uint16_t x)
bool pwmLimit(uint8_t id, uint16_t x)
bool currentLimit(uint8_t id, uint16_t x)
bool accelerationLimit(uint8_t id, uint32_t x)
bool velocityLimit(uint8_t id, uint32_t x)
bool maxPositionLimit(uint8_t id, uint32_t x)
bool minPositionLimit(uint8_t id, uint32_t x)
bool shutdown(uint8_t id, uint8_t x)
bool torqueEnable(uint8_t id, bool x)
bool led(uint8_t id, bool x)
bool statusReturnLevel(uint8_t id, uint8_t x)
bool velocityIGain(uint8_t id, uint16_t x)
bool velocityPGain(uint8_t id, uint16_t x)
bool positionDGain(uint8_t id, uint16_t x)
bool positionIGain(uint8_t id, uint16_t x)
bool positionPGain(uint8_t id, uint16_t x)
bool feedForwardAccelerationGain(uint8_t id, uint16_t x)
bool feedForwardVelocityGain(uint8_t id, uint16_t x)
bool busWatchdog(uint8_t id, int8_t x)
bool goalPwm(uint8_t id, int16_t x)
bool goalCurrent(uint8_t id, int16_t x)
bool goalVelocity(uint8_t id, int32_t x)
bool profileAcceleration(uint8_t id, uint32_t x)
bool profileVelocity(uint8_t id, uint32_t x)
bool goalPosition(uint8_t id, int32_t x)
```

## TBD

- REG_WRITE/ACTION
- INDIRECT ACCESS helper class
- SYNC READ/WRITE
- BULK READ/WRITE


## License

MIT

DynamixelSDK's license is Apache License 2.0.
Please see `Dynamixel/lib/DynamixelSDK` and [DynamixelSDK repo](https://github.com/ROBOTIS-GIT/DynamixelSDK) for details.
