// 2019.04.01 by Hideaki Tai : simplified to adapt only to Arduino
/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel packet control
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_

#include <Arduino.h>
#include "../../../util/ArxTypeTraits/ArxTypeTraits.h"
#include "../../../util/ArxContainer/ArxContainer.h"
#include "types.h"
#include "port_handler_arduino.h"

#define BROADCAST_ID 0xFE  // 254
#define MAX_ID 0xFC        // 252

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b) ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l) ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l) ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING 1
#define INST_READ 2
#define INST_WRITE 3
#define INST_REG_WRITE 4
#define INST_ACTION 5
#define INST_FACTORY_RESET 6
#define INST_SYNC_WRITE 131  // 0x83
#define INST_BULK_READ 146   // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT 8
#define INST_CLEAR 16        // 0x10
#define INST_STATUS 85       // 0x55
#define INST_SYNC_READ 130   // 0x82
#define INST_BULK_WRITE 147  // 0x93

// Communication Result
#define COMM_SUCCESS 0            // tx or rx packet communication success
#define COMM_PORT_BUSY -1000      // Port is busy (in use)
#define COMM_TX_FAIL -1001        // Failed transmit instruction packet
#define COMM_RX_FAIL -1002        // Failed get status packet
#define COMM_TX_ERROR -2000       // Incorrect instruction packet
#define COMM_RX_WAITING -3000     // Now recieving status packet
#define COMM_RX_TIMEOUT -3001     // There is no status packet
#define COMM_RX_CORRUPT -3002     // Incorrect status packet
#define COMM_NOT_AVAILABLE -9000  //

namespace dynamixel {

class PacketHandler {
public:
    PortHandler *port;

    virtual ~PacketHandler() {}

    void attach(PortHandler *p) { port = p; }

    virtual float getProtocolVersion() = 0;

    const char *getTxRxResult(int result) {
#ifndef __AVR__
        switch (result) {
            case COMM_SUCCESS:
                return "[TxRxResult] Communication success.";
            case COMM_PORT_BUSY:
                return "[TxRxResult] Port is in use!";
            case COMM_TX_FAIL:
                return "[TxRxResult] Failed transmit instruction packet!";
            case COMM_RX_FAIL:
                return "[TxRxResult] Failed get status packet from device!";
            case COMM_TX_ERROR:
                return "[TxRxResult] Incorrect instruction packet!";
            case COMM_RX_WAITING:
                return "[TxRxResult] Now recieving status packet!";
            case COMM_RX_TIMEOUT:
                return "[TxRxResult] There is no status packet!";
            case COMM_RX_CORRUPT:
                return "[TxRxResult] Incorrect status packet!";
            case COMM_NOT_AVAILABLE:
                return "[TxRxResult] Protocol does not support This function!";
            default:
                return "";
        }
#else
        (void)result;
        return "";
#endif
    }

    virtual const char *getRxPacketError(uint8_t error) = 0;

    virtual int ping(uint8_t id, uint16_t *model_number, uint8_t *error = 0) = 0;
    virtual int broadcastPing(Vec<uint8_t> &id_list) = 0;
    virtual int reboot(uint8_t id, uint8_t *error = 0) = 0;
    virtual int clearMultiTurn(uint8_t id, uint8_t *error = 0) = 0;
    virtual int factoryReset(uint8_t id, uint8_t option = 0, uint8_t *error = 0) = 0;

    int readBytesTxRx(uint8_t id, uint16_t address, uint8_t *data, uint8_t size, uint8_t *error = 0) {
        uint8_t data_read[size];
        int result = readTxRx(id, address, size, data_read, error);
        if (result == COMM_SUCCESS)
            for (uint8_t i = 0; i < size; ++i) data[i] = data_read[i];
        return result;
    }

    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type * = nullptr>
    int writeBytesTxRx(uint8_t id, uint16_t address, T data, uint8_t size, uint8_t *error = 0) {
        return writeTxRx(id, address, size, (uint8_t *)&data, error);
    }

    virtual int regWriteTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;
    virtual int action(uint8_t id) = 0;

    virtual int readTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;
    virtual int txPacket(uint8_t *txpacket) = 0;
    virtual int rxPacket(uint8_t *rxpacket) = 0;
    virtual int txRxPacket(uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error = 0) = 0;
    virtual int writeTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

    // only for sync_read, bulk_read
    virtual int readRx(uint8_t id, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

private:
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_ */
