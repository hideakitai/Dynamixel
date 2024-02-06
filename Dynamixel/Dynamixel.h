#pragma once
#ifndef ARDUINO_DYNAMIXEL_IMPL_H
#define ARDUINO_DYNAMIXEL_IMPL_H

#include "lib/DynamixelSDK/include/dynamixel_sdk.h"
#include <ArxSmartPtr.h>
#include "DynamixelControlTable.h"

namespace arduino {
namespace dynamixel {

enum class ProtocolVersion {
    V1,
    V2
};

const uint8_t ID_BROADCAST = 0xFE;
const uint8_t ID_LIMIT = 0xFC;

class Dynamixel {
    // TODO: do not use new, make instance inside
    ::dynamixel::PortHandler* port_handler;
    ::dynamixel::PacketHandler* packet_handler;
#if 0
        ::dynamixel::GroupSyncWrite sync_writer;
        ::dynamixel::GroupSyncRead sync_reader;
        ::dynamixel::GroupBulkWrite bulk_writer;
        ::dynamixel::GroupBulkRead bulk_reader;
#endif

    struct Info {
        std::shared_ptr<ControlTable> ct;
        int result;
        uint8_t error;
        uint16_t model;
    };
    Map<uint8_t, Info> info;

public:
    using Models = Vec<uint8_t>;

    // TODO: do not use new, make instance inside
    Dynamixel(uint8_t pin_rts_enable, ProtocolVersion ver = ProtocolVersion::V2)
        : port_handler(new ::dynamixel::PortHandler(pin_rts_enable)), packet_handler((ver == ProtocolVersion::V2) ? (::dynamixel::PacketHandler*)new ::dynamixel::Protocol2PacketHandler() : (::dynamixel::PacketHandler*)new ::dynamixel::Protocol1PacketHandler())
#if 0
        , sync_writer(port_handler, packet_handler)
        , sync_reader(port_handler, packet_handler)
        , bulk_writer(port_handler, packet_handler)
        , bulk_reader(port_handler, packet_handler)
#endif
    {
    }

    Dynamixel(uint8_t pin_rx_enable, uint8_t pin_tx_enable, ProtocolVersion ver = ProtocolVersion::V2)
        : port_handler(new ::dynamixel::PortHandler(pin_rx_enable, pin_tx_enable)), packet_handler((ver == ProtocolVersion::V2) ? (::dynamixel::PacketHandler*)new ::dynamixel::Protocol2PacketHandler() : (::dynamixel::PacketHandler*)new ::dynamixel::Protocol1PacketHandler())
#if 0
        , sync_writer(port_handler, packet_handler)
        , sync_reader(port_handler, packet_handler)
        , bulk_writer(port_handler, packet_handler)
        , bulk_reader(port_handler, packet_handler)
#endif
    {
    }

    ~Dynamixel() {
        delete port_handler;
        delete packet_handler;
    }

    template <Model>
    void addModel(uint8_t id) {
        std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::OTHER>::instance();
        info.emplace(id, Info{sp, 0, 0, 0});
    }

    void attach(Stream& s, size_t baud) {
        port_handler->attach(s, baud);
        packet_handler->attach(port_handler);
    }

    uint8_t size() const { return info.size(); }

    // wrapper for instructions

    bool write(uint8_t id, Reg reg, uint32_t data) {
        if (info.find(id) == info.end()) return false;

        uint16_t addr = info[id].ct->ct[reg].addr;
        uint8_t size = info[id].ct->ct[reg].size;
        uint8_t error;
        int result = packet_handler->writeBytesTxRx(id, addr, data, size, &error);
        return handleResult(id, result, error);
    }

    uint32_t read(uint8_t id, Reg reg) {
        if (info.find(id) == info.end()) return 0xFFFFFFFF;

        uint32_t value = 0;
        uint16_t addr = info[id].ct->ct[reg].addr;
        uint8_t size = info[id].ct->ct[reg].size;
        uint8_t error = 0;
        int result = packet_handler->readBytesTxRx(id, addr, (uint8_t*)&value, size, &error);
        bool b = handleResult(id, result, error);
        return b ? value : 0xFFFFFFFF;
    }

    bool ping(uint8_t id) {
        if (id > ID_LIMIT) return false;
        uint8_t error = 0;
        uint16_t model = 0;
        int result = packet_handler->ping(id, &model, &error);
        return handleResult(id, result, error, model);
    }

    Models ping()  // broadcast
    {
        Models ids;
        int result = packet_handler->broadcastPing(ids);
        if (result != COMM_SUCCESS) return Models();
        return ids;
    }

    bool factoryReset(uint8_t id, ResetMode mode = ResetMode::EXC_ID_BAUD) {
        uint8_t error = 0;
        int result = packet_handler->factoryReset(id, (uint8_t)mode, &error);
        return handleResult(id, result, error);
    }

    bool factoryReset(ResetMode mode = ResetMode::EXC_ID_BAUD) {
        return factoryReset(ID_BROADCAST, mode);
    }

    bool reboot(uint8_t id) {
        uint8_t error = 0;
        int result = packet_handler->reboot(id, &error);
        return handleResult(id, result, error);
    }

    void verbose(uint8_t id) {
        if (info.find(id) == info.end()) return;
        verboseResult(id);
        verboseError(id);
    }

    // TODO: if there is no such ID
    int lastCommResult(uint8_t id) { return (info.find(id) != info.end()) ? info[id].result : -1; }
    uint8_t lastError(uint8_t id) { return (info.find(id) != info.end()) ? info[id].error : 0; }
    uint16_t lastModelNo(uint8_t id) { return (info.find(id) != info.end()) ? info[id].model : 0; }

    // wrappers for control table

    // read values
    uint16_t modelNumber(uint8_t id) { return (uint16_t)read(id, Reg::MODEL_NUMBER); }
    uint32_t modelInformation(uint8_t id) { return (uint32_t)read(id, Reg::MODEL_INFORMATION); }
    uint8_t versionOfFirmware(uint8_t id) { return (uint8_t)read(id, Reg::VERSION_OF_FIRMWARE); }
    uint8_t id(uint8_t id) { return (uint8_t)read(id, Reg::ID); }
    uint8_t baudrate(uint8_t id) { return (uint8_t)read(id, Reg::BAUDRATE); }
    uint8_t returnDelayTime(uint8_t id) { return (uint8_t)read(id, Reg::RETURN_DELAY_TIME); }
    uint8_t driveMode(uint8_t id) { return (uint8_t)read(id, Reg::DRIVE_MODE); }
    uint8_t operatingMode(uint8_t id) { return (uint8_t)read(id, Reg::OPERATING_MODE); }
    uint8_t secondaryId(uint8_t id) { return (uint8_t)read(id, Reg::SECONDARY_ID); }
    uint8_t protocolVersion(uint8_t id) { return (uint8_t)read(id, Reg::PROTOCOL_VERSION); }
    int32_t homingOffset(uint8_t id) { return (int32_t)read(id, Reg::HOMING_OFFSET); }
    uint32_t movingThreshold(uint8_t id) { return (uint32_t)read(id, Reg::MOVING_THRESHOLD); }
    uint8_t temperatureLimit(uint8_t id) { return (uint8_t)read(id, Reg::TEMPERATURE_LIMIT); }
    uint16_t maxVoltageLimit(uint8_t id) { return (uint16_t)read(id, Reg::MAX_VOLTAGE_LIMIT); }
    uint16_t minVoltageLimit(uint8_t id) { return (uint16_t)read(id, Reg::MIN_VOLTAGE_LIMIT); }
    uint16_t pwmLimit(uint8_t id) { return (uint16_t)read(id, Reg::PWM_LIMIT); }
    uint16_t currentLimit(uint8_t id) { return (uint16_t)read(id, Reg::CURRENT_LIMIT); }
    uint32_t accelerationLimit(uint8_t id) { return (uint32_t)read(id, Reg::ACCELERATION_LIMIT); }
    uint32_t velocityLimit(uint8_t id) { return (uint32_t)read(id, Reg::VELOCITY_LIMIT); }
    uint32_t maxPositionLimit(uint8_t id) { return (uint32_t)read(id, Reg::MAX_POSITION_LIMIT); }
    uint32_t minPositionLimit(uint8_t id) { return (uint32_t)read(id, Reg::MIN_POSITION_LIMIT); }
    uint8_t shutdown(uint8_t id) { return (uint8_t)read(id, Reg::SHUTDOWN); }
    bool torqueEnable(uint8_t id) { return (bool)read(id, Reg::TORQUE_ENABLE); }
    uint8_t led(uint8_t id) { return (uint8_t)read(id, Reg::LED); }
    uint8_t statusReturnLevel(uint8_t id) { return (uint8_t)read(id, Reg::STATUS_RETURN_LEVEL); }
    uint8_t registerdInstruction(uint8_t id) { return (uint8_t)read(id, Reg::REGISTERED_INSTRUCTION); }
    uint8_t hardwareErrorStatus(uint8_t id) { return (uint8_t)read(id, Reg::HARDWARE_ERROR_STATUS); }
    uint16_t velocityIGain(uint8_t id) { return (uint16_t)read(id, Reg::VELOCITY_I_GAIN); }
    uint16_t velocityPGain(uint8_t id) { return (uint16_t)read(id, Reg::VELOCITY_P_GAIN); }
    uint16_t positionDGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_D_GAIN); }
    uint16_t positionIGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_I_GAIN); }
    uint16_t positionPGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_P_GAIN); }
    uint16_t feedForwardAccelerationGain(uint8_t id) { return (uint16_t)read(id, Reg::FEEDFORWARD_ACCELERATION_GAIN); }
    uint16_t feedForwardVelocityGain(uint8_t id) { return (uint16_t)read(id, Reg::FEEDFORWARD_VELOCITY_GAIN); }
    int8_t busWatchdog(uint8_t id) { return (int8_t)read(id, Reg::BUS_WATCHDOG); }
    int16_t goalPwm(uint8_t id) { return (int16_t)read(id, Reg::GOAL_PWM); }
    int16_t goalCurrent(uint8_t id) { return (int16_t)read(id, Reg::GOAL_CURRENT); }
    int32_t goalVelocity(uint8_t id) { return (int32_t)read(id, Reg::GOAL_VELOCITY); }
    uint32_t profileAcceleration(uint8_t id) { return (uint32_t)read(id, Reg::PROFILE_ACCELERATION); }
    uint32_t profileVelocity(uint8_t id) { return (uint32_t)read(id, Reg::PROFILE_VELOCITY); }
    int32_t goalPosition(uint8_t id) { return (int32_t)read(id, Reg::GOAL_POSITION); }
    uint16_t realTimeTick(uint8_t id) { return (uint16_t)read(id, Reg::REALTIME_TICK); }
    uint8_t moving(uint8_t id) { return (uint8_t)read(id, Reg::MOVING); }
    uint8_t movingStatus(uint8_t id) { return (uint8_t)read(id, Reg::MOVING_STATUS); }
    int16_t presentPwm(uint8_t id) { return (int16_t)read(id, Reg::PRESENT_PWM); }
    int16_t presentCurrent(uint8_t id) { return (int16_t)read(id, Reg::PRESENT_CURRENT); }
    int32_t presentVelocity(uint8_t id) { return (int32_t)read(id, Reg::PRESENT_VELOCITY); }
    int32_t presentPosition(uint8_t id) { return (int32_t)read(id, Reg::PRESENT_POSITION); }
    uint32_t velocityTrajectory(uint8_t id) { return (uint32_t)read(id, Reg::VELOCITY_TRAJECTORY); }
    uint32_t positionTrajectory(uint8_t id) { return (uint32_t)read(id, Reg::POSITION_TRAJECTORY); }
    uint16_t presentInputVoltage(uint8_t id) { return (uint16_t)read(id, Reg::PRESENT_INPUT_VOLTAGE); }
    uint8_t presentTemperature(uint8_t id) { return (uint8_t)read(id, Reg::PRESENT_TEMPERATURE); }

    // write values
    bool id(uint8_t id, uint8_t x) { return write(id, Reg::ID, x); }
    bool baudrate(uint8_t id, uint8_t x) { return write(id, Reg::BAUDRATE, x); }
    bool returnDelayTime(uint8_t id, uint8_t x) { return write(id, Reg::RETURN_DELAY_TIME, x); }
    bool driveMode(uint8_t id, uint8_t x) { return write(id, Reg::DRIVE_MODE, x); }
    bool operatingMode(uint8_t id, uint8_t x) { return write(id, Reg::OPERATING_MODE, x); }
    bool secondaryId(uint8_t id, uint8_t x) { return write(id, Reg::SECONDARY_ID, x); }
    bool protocolVersion(uint8_t id, uint8_t x) { return write(id, Reg::PROTOCOL_VERSION, x); }
    bool homingOffset(uint8_t id, int32_t x) { return write(id, Reg::HOMING_OFFSET, x); }
    bool movingThreshold(uint8_t id, uint32_t x) { return write(id, Reg::MOVING_THRESHOLD, x); }
    bool temperatureLimit(uint8_t id, uint8_t x) { return write(id, Reg::TEMPERATURE_LIMIT, x); }
    bool maxVoltageLimit(uint8_t id, uint16_t x) { return write(id, Reg::MAX_VOLTAGE_LIMIT, x); }
    bool minVoltageLimit(uint8_t id, uint16_t x) { return write(id, Reg::MIN_VOLTAGE_LIMIT, x); }
    bool pwmLimit(uint8_t id, uint16_t x) { return write(id, Reg::PWM_LIMIT, x); }
    bool currentLimit(uint8_t id, uint16_t x) { return write(id, Reg::CURRENT_LIMIT, x); }
    bool accelerationLimit(uint8_t id, uint32_t x) { return write(id, Reg::ACCELERATION_LIMIT, x); }
    bool velocityLimit(uint8_t id, uint32_t x) { return write(id, Reg::VELOCITY_LIMIT, x); }
    bool maxPositionLimit(uint8_t id, uint32_t x) { return write(id, Reg::MAX_POSITION_LIMIT, x); }
    bool minPositionLimit(uint8_t id, uint32_t x) { return write(id, Reg::MIN_POSITION_LIMIT, x); }
    bool shutdown(uint8_t id, uint8_t x) { return write(id, Reg::SHUTDOWN, x); }
    bool torqueEnable(uint8_t id, bool x) { return write(id, Reg::TORQUE_ENABLE, x); }
    bool led(uint8_t id, bool x) { return write(id, Reg::LED, x); }
    bool statusReturnLevel(uint8_t id, uint8_t x) { return write(id, Reg::STATUS_RETURN_LEVEL, x); }
    bool velocityIGain(uint8_t id, uint16_t x) { return write(id, Reg::VELOCITY_I_GAIN, x); }
    bool velocityPGain(uint8_t id, uint16_t x) { return write(id, Reg::VELOCITY_P_GAIN, x); }
    bool positionDGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_D_GAIN, x); }
    bool positionIGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_I_GAIN, x); }
    bool positionPGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_P_GAIN, x); }
    bool feedForwardAccelerationGain(uint8_t id, uint16_t x) { return write(id, Reg::FEEDFORWARD_ACCELERATION_GAIN, x); }
    bool feedForwardVelocityGain(uint8_t id, uint16_t x) { return write(id, Reg::FEEDFORWARD_VELOCITY_GAIN, x); }
    bool busWatchdog(uint8_t id, int8_t x) { return write(id, Reg::BUS_WATCHDOG, x); }
    bool goalPwm(uint8_t id, int16_t x) { return write(id, Reg::GOAL_PWM, x); }
    bool goalCurrent(uint8_t id, int16_t x) { return write(id, Reg::GOAL_CURRENT, x); }
    bool goalVelocity(uint8_t id, int32_t x) { return write(id, Reg::GOAL_VELOCITY, x); }
    bool profileAcceleration(uint8_t id, uint32_t x) { return write(id, Reg::PROFILE_ACCELERATION, x); }
    bool profileVelocity(uint8_t id, uint32_t x) { return write(id, Reg::PROFILE_VELOCITY, x); }
    bool goalPosition(uint8_t id, int32_t x) { return write(id, Reg::GOAL_POSITION, x); }

private:
    bool handleResult(uint8_t id, uint8_t result, uint8_t error = 0, uint16_t model = 0) {
        bool b = true;
        if ((result != COMM_SUCCESS) || (error != 0)) b = false;
        saveResult(id, result, error, model);
        return b;
    }

    void saveResult(uint8_t id, int result, uint8_t error = 0, uint16_t model = 0) {
        if (id > ID_LIMIT) return;

        auto it = info.find(id);
        if (it == info.end()) return;

        it->second.result = result;
        it->second.error = error;
        if (model != 0) it->second.model = model;
    }

    void verboseResult(uint8_t id) {
        if (info[id].result != COMM_SUCCESS)
            Serial.println(packet_handler->getTxRxResult(info[id].result));
    }
    void verboseError(uint8_t id) {
        if (info[id].error != 0)
            Serial.println(packet_handler->getRxPacketError(info[id].error));
    }

#if 0   // TODO: reg_write & action, indirect, sync, bulk
        bool reg_write(uint8_t id, uint16_t reg, uint16_t size, uint8_t* data)
        {
            // TODO: clamp value
            uint8_t error = 0;
            int result = packet_handler->regWriteTxRx(id, reg, size, data, &error);
            return handleResult(id, result, error);
        }
        bool action(uint8_t id)
        {
            int result = packet_handler->action(id);
            return handleResult(id, result);
        }
        // for indirect address
        bool setIndirectAddress(uint8_t id, uint8_t nth, uint16_t addr)
        {
            if (nth > 128) return false;
            uint8_t error = 0;
            // int result = packet_handler->write2ByteTxRx(id, info[id].ct->ct[Reg::INDIRECT_ADDR_1].addr + 2 * nth, addr, &error);
            int result = packet_handler->writeBytesTxRx(id, info[id].ct->ct[Reg::INDIRECT_ADDR_1].addr + 2 * nth, addr, info[id].ct->ct[Reg::INDIRECT_ADDR_1].size, &error);
            return handleResult(id, result, error);
        }
        bool setIndirectData(uint8_t id, uint8_t nth, uint8_t data)
        {
            uint8_t error = 0;
            // TODO: ayashii data size
            // int result = packet_handler->write2ByteTxRx(id, info[id].ct->ct[Reg::INDIRECT_DATA_1].addr + nth, data, &error);
            int result = packet_handler->writeBytesTxRx(id, info[id].ct->ct[Reg::INDIRECT_DATA_1].addr + nth, data, info[id].ct->ct[Reg::INDIRECT_DATA_1].size, &error);
            return handleResult(id, result, error);
        }
        // for sync write
        void set_address(uint16_t addr, uint16_t size)
        {
            sync_writer.setAddress(addr, size);
        }
        bool add(uint8_t id, uint8_t* data)
        {
            // TODO: check if endian is ok or not
            int result = sync_writer.addParam(id, data);
            if (result != true) return false;
            return true;
        }
        bool send()
        {
            int result = sync_writer.txPacket();
            sync_writer.clearParam();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        // for sync read
        // TODO: duplicate name with sync write
        void set_target(uint16_t addr, uint16_t size)
        {
            sync_reader.setAddress(addr, size);
        }
        // TODO: varidic arguments...
        bool add_id(uint8_t id)
        {
            int result = sync_reader.addParam(id);
            if (result != true) return false;
            return true;
        }
        bool request()
        {
            int result = sync_reader.txRxPacket();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        bool available(uint8_t id)
        {
            return sync_reader.isAvailable(id);
        }
        uint32_t data(uint8_t id)
        {
            return sync_reader.getData(id);
        }
        // for bulk write
        bool add_bulk_target(uint8_t id, uint16_t addr, uint16_t size, uint8_t* data)
        {
            return bulk_writer.addParam(id, addr, size, data);
        }
        bool bulk_write()
        {
            int result = bulk_writer.txPacket();
            bulk_writer.clearParam();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        // for bulk read
        bool add_bulk_read_target(uint8_t id, uint16_t addr, uint16_t size)
        {
            return bulk_reader.addParam(id, addr, size);
        }
        bool bulk_read_request()
        {
            int result = bulk_reader.txRxPacket();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        bool bulk_available(uint8_t id)
        {
            return bulk_reader.isAvailable(id);
        }
        uint32_t bulk_read_data(uint8_t id)
        {
            return bulk_reader.getData(id);
        }
#endif  // TODO:
};

template <>
inline void Dynamixel::addModel<Model::PRO>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::PRO>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}
template <>
inline void Dynamixel::addModel<Model::X>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::X>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}
template <>
inline void Dynamixel::addModel<Model::MX>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::MX>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}

}  // namespace dynamixel
}  // namespace arduino

using Dynamixel = arduino::dynamixel::Dynamixel;
using DxlModel = arduino::dynamixel::Model;
using DxlReg = arduino::dynamixel::Reg;

#endif  // ARDUINO_DYNAMIXEL_IMPL_H
