#pragma once
#ifndef DYNAMIXEL_CONTROL_TABLE_H
#define DYNAMIXEL_CONTROL_TABLE_H

#include "util/ArxSmartPtr/ArxSmartPtr.h"

namespace arduino {
namespace dynamixel {

#if ARX_HAVE_LIBSTDCPLUSPLUS >= 201103L  // Have libstdc++11
template <typename T>
using Vec = std::vector<T>;
template <typename T, typename U>
using Map = std::map<T, U>;
#else  // Do not have libstdc++11
template <typename T>
using Vec = arx::vector<T>;
template <typename T, typename U>
using Map = arx::map<T, U>;
#endif

enum class Model {
    PRO,
    X,
    MX,
    OTHER
};

enum class Reg {
    // EEPROM
    MODEL_NUMBER,
    MODEL_INFORMATION,
    VERSION_OF_FIRMWARE,
    ID,
    BAUDRATE,
    RETURN_DELAY_TIME,
    DRIVE_MODE,
    OPERATING_MODE,
    SECONDARY_ID,
    PROTOCOL_VERSION,
    HOMING_OFFSET,
    MOVING_THRESHOLD,
    TEMPERATURE_LIMIT,
    MAX_VOLTAGE_LIMIT,
    MIN_VOLTAGE_LIMIT,
    PWM_LIMIT,
    CURRENT_LIMIT,
    ACCELERATION_LIMIT,
    VELOCITY_LIMIT,
    MAX_POSITION_LIMIT,
    MIN_POSITION_LIMIT,
    SHUTDOWN,
    // RAM
    TORQUE_ENABLE,
    LED,
    STATUS_RETURN_LEVEL,
    REGISTERED_INSTRUCTION,
    HARDWARE_ERROR_STATUS,
    VELOCITY_I_GAIN,
    VELOCITY_P_GAIN,
    POSITION_D_GAIN,
    POSITION_I_GAIN,
    POSITION_P_GAIN,
    FEEDFORWARD_ACCELERATION_GAIN,
    FEEDFORWARD_VELOCITY_GAIN,
    BUS_WATCHDOG,
    GOAL_PWM,
    GOAL_CURRENT,
    GOAL_VELOCITY,
    PROFILE_ACCELERATION,
    PROFILE_VELOCITY,
    GOAL_POSITION,
    REALTIME_TICK,
    MOVING,
    MOVING_STATUS,
    PRESENT_PWM,
    PRESENT_CURRENT,
    PRESENT_VELOCITY,
    PRESENT_POSITION,
    VELOCITY_TRAJECTORY,
    POSITION_TRAJECTORY,

    // TBD

    PRESENT_INPUT_VOLTAGE,
    PRESENT_TEMPERATURE,
    INDIRECT_ADDR_1,
    INDIRECT_DATA_1,
    INDIRECT_ADDR_29,
    INDIRECT_DATA_29,

    // EEPROM (OTHERS)
    // CW_ANGLE_LIMIT,
    // CCW_ANGLE_LIMIT,
    TORQUE_MAX,
    ALARM_LED,
    ALARM_SHUTDOWN,
    MULTI_TURN_OFFSET,
    RESOLUTION_DIVIDER,
    TORQUE_LIMIT,
    EEPROM_LOCK,
    PUNCH,
    GOAL_ACCELERATION,

    EXTERNAL_PORT_MODE_1,
    EXTERNAL_PORT_MODE_2,
    EXTERNAL_PORT_MODE_3,
    EXTERNAL_PORT_MODE_4,
    EXTERNAL_PORT_DATA_1,
    EXTERNAL_PORT_DATA_2,
    EXTERNAL_PORT_DATA_3,
    EXTERNAL_PORT_DATA_4,
    LED_R,
    LED_G,
    LED_B,
    GOAL_TORQUE,
    PRESENT_LOAD,

    DOWN_CALIBRATION,
    UP_CALIBRATION,
    CW_COMPLIANCE_MARGIN,
    CCW_COMPLIANCE_MARGIN,
    CW_COMPLIANCE_SLOPE,
    CCW_COMPLIANCE_SLOPE,

};

struct RegInfo {
    uint16_t addr;
    uint8_t size;
};

struct ControlTable {
    Map<Reg, RegInfo> ct;
};

template <Model>  // Model::OTHER
class ControlTableOfModel : public ControlTable {
public:
    static std::shared_ptr<ControlTableOfModel> instance() {
        static std::shared_ptr<ControlTableOfModel> instance(new ControlTableOfModel());
        return instance;
    }

private:
    ControlTableOfModel(const ControlTableOfModel&) = delete;
    ControlTableOfModel& operator=(const ControlTableOfModel&) = delete;
    ControlTableOfModel() {
        Serial.println(F("OTHER series Control Table"));
        ct = Map<Reg, RegInfo>{
            // only for Protocol 1.0
            // EEPROM
            {Reg::MODEL_NUMBER, {0, 2}},
            {Reg::VERSION_OF_FIRMWARE, {2, 1}},
            {Reg::ID, {3, 1}},
            {Reg::BAUDRATE, {4, 1}},
            {Reg::RETURN_DELAY_TIME, {5, 1}},
            {Reg::MIN_POSITION_LIMIT, {6, 2}},
            {Reg::MAX_POSITION_LIMIT, {8, 2}},
            {Reg::TEMPERATURE_LIMIT, {11, 1}},
            {Reg::MIN_VOLTAGE_LIMIT, {12, 1}},
            {Reg::MAX_VOLTAGE_LIMIT, {13, 1}},
            {Reg::TORQUE_MAX, {14, 2}},
            {Reg::STATUS_RETURN_LEVEL, {16, 1}},
            {Reg::ALARM_LED, {17, 1}},
            {Reg::ALARM_SHUTDOWN, {18, 1}},
            {Reg::DOWN_CALIBRATION, {20, 2}},
            {Reg::UP_CALIBRATION, {22, 2}},
            {Reg::TORQUE_ENABLE, {24, 1}},
            {Reg::LED, {25, 1}},
            {Reg::CW_COMPLIANCE_MARGIN, {26, 1}},
            {Reg::CCW_COMPLIANCE_MARGIN, {27, 1}},
            {Reg::CW_COMPLIANCE_SLOPE, {28, 1}},
            {Reg::CCW_COMPLIANCE_SLOPE, {29, 1}},
            {Reg::GOAL_POSITION, {30, 2}},
            {Reg::VELOCITY_LIMIT, {32, 2}},
            {Reg::TORQUE_LIMIT, {34, 2}},
            {Reg::PRESENT_POSITION, {36, 2}},
            {Reg::PRESENT_VELOCITY, {38, 2}},
            {Reg::PRESENT_LOAD, {40, 2}},
            {Reg::PRESENT_INPUT_VOLTAGE, {42, 1}},
            {Reg::PRESENT_TEMPERATURE, {43, 1}},
            {Reg::REGISTERED_INSTRUCTION, {44, 1}},
            {Reg::MOVING, {46, 1}},
            {Reg::EEPROM_LOCK, {47, 1}},
            {Reg::PUNCH, {48, 2}},
        };
    }
};

template <>
class ControlTableOfModel<Model::PRO> : public ControlTable {
public:
    static std::shared_ptr<ControlTableOfModel> instance() {
        static std::shared_ptr<ControlTableOfModel> instance(new ControlTableOfModel());
        return instance;
    }

private:
    ControlTableOfModel(const ControlTableOfModel&) = delete;
    ControlTableOfModel& operator=(const ControlTableOfModel&) = delete;
    ControlTableOfModel() {
        Serial.println(F("PRO series Control Table"));
        ct = Map<Reg, RegInfo>{
            {Reg::MODEL_NUMBER, {0, 2}},
            {Reg::MODEL_INFORMATION, {2, 4}},
            {Reg::VERSION_OF_FIRMWARE, {6, 1}},
            {Reg::ID, {7, 1}},
            {Reg::BAUDRATE, {8, 1}},
            {Reg::RETURN_DELAY_TIME, {9, 1}},
            {Reg::OPERATING_MODE, {11, 1}},
            {Reg::HOMING_OFFSET, {13, 4}},
            {Reg::MOVING_THRESHOLD, {17, 4}},
            {Reg::TEMPERATURE_LIMIT, {21, 1}},
            {Reg::MAX_VOLTAGE_LIMIT, {22, 2}},
            {Reg::MIN_VOLTAGE_LIMIT, {24, 2}},
            {Reg::ACCELERATION_LIMIT, {26, 4}},
            {Reg::TORQUE_LIMIT, {30, 2}},
            {Reg::VELOCITY_LIMIT, {32, 4}},
            {Reg::MAX_POSITION_LIMIT, {36, 4}},
            {Reg::MIN_POSITION_LIMIT, {40, 4}},
            {Reg::EXTERNAL_PORT_MODE_1, {44, 1}},
            {Reg::EXTERNAL_PORT_MODE_2, {45, 1}},
            {Reg::EXTERNAL_PORT_MODE_3, {46, 1}},
            {Reg::EXTERNAL_PORT_MODE_4, {47, 1}},
            {Reg::SHUTDOWN, {48, 1}},
            {Reg::INDIRECT_ADDR_1, {49, 2}},
            {Reg::TORQUE_ENABLE, {562, 1}},
            {Reg::LED_R, {563, 1}},
            {Reg::LED_G, {564, 1}},
            {Reg::LED_B, {565, 1}},
            {Reg::VELOCITY_I_GAIN, {586, 2}},
            {Reg::VELOCITY_P_GAIN, {588, 2}},
            {Reg::POSITION_P_GAIN, {594, 2}},
            {Reg::GOAL_POSITION, {596, 4}},
            {Reg::GOAL_VELOCITY, {600, 4}},
            {Reg::GOAL_TORQUE, {604, 2}},
            {Reg::GOAL_ACCELERATION, {606, 4}},
            {Reg::MOVING, {610, 1}},
            {Reg::PRESENT_POSITION, {611, 4}},
            {Reg::PRESENT_VELOCITY, {615, 4}},
            {Reg::PRESENT_CURRENT, {621, 2}},
            {Reg::PRESENT_INPUT_VOLTAGE, {623, 2}},
            {Reg::PRESENT_TEMPERATURE, {625, 1}},
            {Reg::EXTERNAL_PORT_DATA_1, {626, 2}},
            {Reg::EXTERNAL_PORT_DATA_2, {628, 2}},
            {Reg::EXTERNAL_PORT_DATA_3, {630, 2}},
            {Reg::EXTERNAL_PORT_DATA_4, {632, 2}},
            {Reg::INDIRECT_DATA_1, {634, 1}},
            {Reg::REGISTERED_INSTRUCTION, {890, 1}},
            {Reg::STATUS_RETURN_LEVEL, {891, 1}},
            {Reg::HARDWARE_ERROR_STATUS, {892, 1}},
        };
    }
};

template <>
class ControlTableOfModel<Model::X> : public ControlTable {
public:
    static std::shared_ptr<ControlTableOfModel> instance() {
        static std::shared_ptr<ControlTableOfModel> instance(new ControlTableOfModel());
        return instance;
    }

private:
    ControlTableOfModel(const ControlTableOfModel&) = delete;
    ControlTableOfModel& operator=(const ControlTableOfModel&) = delete;
    ControlTableOfModel() {
        Serial.println(F("X series Control Table"));
        ct = Map<Reg, RegInfo>{
            // EEPROM
            {Reg::MODEL_NUMBER, {0, 2}},
            {Reg::MODEL_INFORMATION, {2, 4}},
            {Reg::VERSION_OF_FIRMWARE, {6, 1}},
            {Reg::ID, {7, 1}},
            {Reg::BAUDRATE, {8, 1}},
            {Reg::RETURN_DELAY_TIME, {9, 1}},
            {Reg::DRIVE_MODE, {10, 1}},
            {Reg::OPERATING_MODE, {11, 1}},
            {Reg::SECONDARY_ID, {12, 1}},
            {Reg::PROTOCOL_VERSION, {13, 1}},
            {Reg::HOMING_OFFSET, {20, 4}},
            {Reg::MOVING_THRESHOLD, {24, 4}},
            {Reg::TEMPERATURE_LIMIT, {31, 1}},
            {Reg::MAX_VOLTAGE_LIMIT, {32, 2}},
            {Reg::MIN_VOLTAGE_LIMIT, {34, 2}},
            {Reg::PWM_LIMIT, {36, 2}},
            {Reg::CURRENT_LIMIT, {38, 2}},
            {Reg::ACCELERATION_LIMIT, {40, 4}},
            {Reg::VELOCITY_LIMIT, {44, 4}},
            {Reg::MAX_POSITION_LIMIT, {48, 4}},
            {Reg::MIN_POSITION_LIMIT, {52, 4}},
            {Reg::SHUTDOWN, {63, 4}},
            // RAM
            {Reg::TORQUE_ENABLE, {64, 1}},
            {Reg::LED, {65, 1}},
            {Reg::STATUS_RETURN_LEVEL, {68, 1}},
            {Reg::REGISTERED_INSTRUCTION, {69, 1}},  // 0: no instruction, 1: reg_write && !action
            {Reg::HARDWARE_ERROR_STATUS, {70, 1}},
            {Reg::VELOCITY_I_GAIN, {76, 2}},
            {Reg::VELOCITY_P_GAIN, {78, 2}},
            {Reg::POSITION_D_GAIN, {80, 2}},
            {Reg::POSITION_I_GAIN, {82, 2}},
            {Reg::POSITION_P_GAIN, {84, 2}},
            {Reg::FEEDFORWARD_ACCELERATION_GAIN, {88, 2}},
            {Reg::FEEDFORWARD_VELOCITY_GAIN, {90, 2}},
            {Reg::BUS_WATCHDOG, {98, 1}},
            {Reg::GOAL_PWM, {100, 2}},
            {Reg::GOAL_CURRENT, {102, 2}},
            {Reg::GOAL_VELOCITY, {104, 4}},
            {Reg::PROFILE_ACCELERATION, {108, 4}},
            {Reg::PROFILE_VELOCITY, {112, 4}},
            {Reg::GOAL_POSITION, {116, 4}},
            {Reg::REALTIME_TICK, {120, 2}},
            {Reg::MOVING, {122, 1}},
            {Reg::MOVING_STATUS, {123, 1}},
            {Reg::PRESENT_PWM, {124, 2}},
            {Reg::PRESENT_CURRENT, {126, 2}},
            {Reg::PRESENT_VELOCITY, {128, 4}},
            {Reg::PRESENT_POSITION, {132, 4}},
            {Reg::VELOCITY_TRAJECTORY, {136, 4}},
            {Reg::POSITION_TRAJECTORY, {140, 4}},
            {Reg::PRESENT_INPUT_VOLTAGE, {144, 2}},
            {Reg::PRESENT_TEMPERATURE, {146, 1}},
            {Reg::INDIRECT_ADDR_1, {168, 2}},
            {Reg::INDIRECT_DATA_1, {224, 1}},
            {Reg::INDIRECT_ADDR_29, {578, 2}},
            {Reg::INDIRECT_DATA_29, {634, 1}}};
    }
};

template <>
class ControlTableOfModel<Model::MX> : public ControlTable {
public:
    static std::shared_ptr<ControlTableOfModel> instance() {
        static std::shared_ptr<ControlTableOfModel> instance(new ControlTableOfModel());
        return instance;
    }

private:
    ControlTableOfModel(const ControlTableOfModel&) = delete;
    ControlTableOfModel& operator=(const ControlTableOfModel&) = delete;
    ControlTableOfModel() {
        Serial.println(F("MX series Control Table"));
        ct = Map<Reg, RegInfo>{
            // only for Protocol 2.0
            // EEPROM
            {Reg::MODEL_NUMBER, {0, 2}},
            {Reg::MODEL_INFORMATION, {2, 4}},
            {Reg::VERSION_OF_FIRMWARE, {6, 1}},
            {Reg::ID, {7, 1}},
            {Reg::BAUDRATE, {8, 1}},
            {Reg::RETURN_DELAY_TIME, {9, 1}},
            {Reg::DRIVE_MODE, {10, 1}},
            {Reg::OPERATING_MODE, {11, 1}},
            {Reg::SECONDARY_ID, {12, 1}},
            {Reg::PROTOCOL_VERSION, {13, 1}},
            {Reg::HOMING_OFFSET, {20, 4}},
            {Reg::MOVING_THRESHOLD, {24, 4}},
            {Reg::TEMPERATURE_LIMIT, {31, 1}},
            {Reg::MAX_VOLTAGE_LIMIT, {32, 2}},
            {Reg::MIN_VOLTAGE_LIMIT, {34, 2}},
            {Reg::PWM_LIMIT, {36, 2}},
            {Reg::CURRENT_LIMIT, {38, 2}},
            {Reg::ACCELERATION_LIMIT, {40, 4}},
            {Reg::VELOCITY_LIMIT, {44, 4}},
            {Reg::MAX_POSITION_LIMIT, {48, 4}},
            {Reg::MIN_POSITION_LIMIT, {52, 4}},
            {Reg::SHUTDOWN, {63, 4}},
            // RAM
            {Reg::TORQUE_ENABLE, {64, 1}},
            {Reg::LED, {65, 1}},
            {Reg::STATUS_RETURN_LEVEL, {68, 1}},
            {Reg::REGISTERED_INSTRUCTION, {69, 1}},  // 0: no instruction, 1: reg_write && !action
            {Reg::HARDWARE_ERROR_STATUS, {70, 1}},
            {Reg::VELOCITY_I_GAIN, {76, 2}},
            {Reg::VELOCITY_P_GAIN, {78, 2}},
            {Reg::POSITION_D_GAIN, {80, 2}},
            {Reg::POSITION_I_GAIN, {82, 2}},
            {Reg::POSITION_P_GAIN, {84, 2}},
            {Reg::FEEDFORWARD_ACCELERATION_GAIN, {88, 2}},
            {Reg::FEEDFORWARD_VELOCITY_GAIN, {90, 2}},
            {Reg::BUS_WATCHDOG, {98, 1}},
            {Reg::GOAL_PWM, {100, 2}},
            {Reg::GOAL_CURRENT, {102, 2}},
            {Reg::GOAL_VELOCITY, {104, 4}},
            {Reg::PROFILE_ACCELERATION, {108, 4}},
            {Reg::PROFILE_VELOCITY, {112, 4}},
            {Reg::GOAL_POSITION, {116, 4}},
            {Reg::REALTIME_TICK, {120, 2}},
            {Reg::MOVING, {122, 1}},
            {Reg::MOVING_STATUS, {123, 1}},
            {Reg::PRESENT_PWM, {124, 2}},
            {Reg::PRESENT_CURRENT, {126, 2}},
            {Reg::PRESENT_VELOCITY, {128, 4}},
            {Reg::PRESENT_POSITION, {132, 4}},
            {Reg::VELOCITY_TRAJECTORY, {136, 4}},
            {Reg::POSITION_TRAJECTORY, {140, 4}},
            {Reg::PRESENT_INPUT_VOLTAGE, {144, 2}},
            {Reg::PRESENT_TEMPERATURE, {146, 1}},
            {Reg::EXTERNAL_PORT_DATA_1, {152, 2}},
            {Reg::EXTERNAL_PORT_DATA_2, {154, 2}},
            {Reg::EXTERNAL_PORT_DATA_3, {156, 2}},
            {Reg::INDIRECT_ADDR_1, {168, 2}},
            {Reg::INDIRECT_DATA_1, {224, 1}},
            {Reg::INDIRECT_ADDR_29, {578, 2}},
            {Reg::INDIRECT_DATA_29, {634, 1}}};
    }
};

enum class SatusReturnLevel {
    EXC_PING,
    EXC_PING_READ,
    ALL
};
enum class ResetMode {
    ALL = 0xFF,
    EXC_ID = 0x01,
    EXC_ID_BAUD = 0x02
};

}  // namespace dynamixel
}  // namespace arduino

#endif  // DYNAMIXEL_CONTROL_TABLE_H
