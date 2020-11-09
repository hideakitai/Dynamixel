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
/// @file The file for Protocol 1.0 Dynamixel packet control
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_

#include "types.h"
#include "packet_handler.h"

namespace dynamixel {

class Protocol1PacketHandler : public PacketHandler {
    static const size_t TXPACKET_MAX_LEN = 250;
    static const size_t RXPACKET_MAX_LEN = 250;

    ///////////////// for Protocol 1.0 Packet /////////////////
    static const uint8_t PKT_HEADER0 = 0;
    static const uint8_t PKT_HEADER1 = 1;
    static const uint8_t PKT_ID = 2;
    static const uint8_t PKT_LENGTH = 3;
    static const uint8_t PKT_INSTRUCTION = 4;
    static const uint8_t PKT_ERROR = 4;
    static const uint8_t PKT_PARAMETER0 = 5;

    ///////////////// Protocol 1.0 Error bit /////////////////
    static const uint8_t ERRBIT_VOLTAGE = 1;       // Supplied voltage is out of the range (operating volatage set in the control table)
    static const uint8_t ERRBIT_ANGLE = 2;         // Goal position is written out of the range (from CW angle limit to CCW angle limit)
    static const uint8_t ERRBIT_OVERHEAT = 4;      // Temperature is out of the range (operating temperature set in the control table)
    static const uint8_t ERRBIT_RANGE = 8;         // Command(setting value) is out of the range for use.
    static const uint8_t ERRBIT_CHECKSUM = 16;     // Instruction packet checksum is incorrect.
    static const uint8_t ERRBIT_OVERLOAD = 32;     // The current load cannot be controlled by the set torque.
    static const uint8_t ERRBIT_INSTRUCTION = 64;  // Undefined instruction or delivering the action command without the reg_write command.

public:
    virtual ~Protocol1PacketHandler() {}

    float getProtocolVersion() { return 1.0; }

    const char *getRxPacketError(uint8_t error) {
#ifndef __AVR__
        if (error & ERRBIT_VOLTAGE) return "[RxPacketError] Input voltage error!";
        if (error & ERRBIT_ANGLE) return "[RxPacketError] Angle limit error!";
        if (error & ERRBIT_OVERHEAT) return "[RxPacketError] Overheat error!";
        if (error & ERRBIT_RANGE) return "[RxPacketError] Out of range error!";
        if (error & ERRBIT_CHECKSUM) return "[RxPacketError] Checksum error!";
        if (error & ERRBIT_OVERLOAD) return "[RxPacketError] Overload error!";
        if (error & ERRBIT_INSTRUCTION) return "[RxPacketError] Instruction code error!";
#else
        (void)error;
#endif
        return "";
    }

    int txPacket(uint8_t *txpacket) {
        uint8_t checksum = 0;
        uint16_t total_packet_length = txpacket[PKT_LENGTH] + 4;  // 4: HEADER0 HEADER1 ID LENGTH
        uint8_t written_packet_length = 0;

        // check max packet length
        if (total_packet_length > TXPACKET_MAX_LEN) return COMM_TX_ERROR;

        // make packet header
        txpacket[PKT_HEADER0] = 0xFF;
        txpacket[PKT_HEADER1] = 0xFF;

        // add a checksum to the packet
        for (uint16_t idx = 2; idx < total_packet_length - 1; idx++)  // except header, checksum
            checksum += txpacket[idx];
        txpacket[total_packet_length - 1] = ~checksum;

        // tx packet
        port->clearPort();
        written_packet_length = port->writePort(txpacket, total_packet_length);
        if (total_packet_length != written_packet_length)
            return COMM_TX_FAIL;

        return COMM_SUCCESS;
    }

    int rxPacket(uint8_t *rxpacket) {
        int result = COMM_TX_FAIL;
        uint8_t checksum = 0;
        uint16_t rx_length = 0;
        uint16_t wait_length = 6;  // minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while (true) {
            rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
            if (rx_length >= wait_length) {
                uint8_t idx = 0;

                // find packet header
                for (idx = 0; idx < (rx_length - 1); idx++) {
                    if (rxpacket[idx] == 0xFF && rxpacket[idx + 1] == 0xFF)
                        break;
                }

                if (idx == 0)  // found at the beginning of the packet
                {
                    if (rxpacket[PKT_ID] > 0xFD ||                  // unavailable ID
                        rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN ||  // unavailable Length
                        rxpacket[PKT_ERROR] > 0x7F)                 // unavailable Error
                    {
                        // remove the first byte in the packet
                        for (uint16_t s = 0; s < rx_length - 1; s++)
                            rxpacket[s] = rxpacket[1 + s];
                        rx_length -= 1;
                        continue;
                    }

                    // re-calculate the exact length of the rx packet
                    if (wait_length != (uint16_t)(rxpacket[PKT_LENGTH] + PKT_LENGTH + 1)) {
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
                        continue;
                    }

                    if (rx_length < wait_length) {
                        // check timeout
                        if (port->isPacketTimeout() == true) {
                            if (rx_length == 0)
                                result = COMM_RX_TIMEOUT;
                            else
                                result = COMM_RX_CORRUPT;
                            break;
                        } else
                            continue;
                    }

                    // calculate checksum
                    for (uint16_t i = 2; i < wait_length - 1; i++)  // except header, checksum
                        checksum += rxpacket[i];
                    checksum = ~checksum;

                    // verify checksum
                    if (rxpacket[wait_length - 1] == checksum)
                        result = COMM_SUCCESS;
                    else
                        result = COMM_RX_CORRUPT;
                    break;
                } else {
                    // remove unnecessary packets
                    for (uint16_t s = 0; s < rx_length - idx; s++)
                        rxpacket[s] = rxpacket[idx + s];
                    rx_length -= idx;
                }
            } else {
                if (port->isPacketTimeout() == true) {
                    if (rx_length == 0)
                        result = COMM_RX_TIMEOUT;
                    else
                        result = COMM_RX_CORRUPT;
                    break;
                }
            }
        }

        return result;
    }

    int txRxPacket(uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;

        // tx packet
        result = txPacket(txpacket);
        if (result != COMM_SUCCESS)
            return result;

        // (Instruction == BulkRead) == this function is not available.
        if (txpacket[PKT_INSTRUCTION] == INST_BULK_READ)
            result = COMM_NOT_AVAILABLE;

        // (ID == Broadcast ID) == no need to wait for status packet or not available
        // (Instruction == action) == no need to wait for status packet
        if (txpacket[PKT_ID] == BROADCAST_ID || txpacket[PKT_INSTRUCTION] == INST_ACTION)
            return result;

        // set packet timeout
        if (txpacket[PKT_INSTRUCTION] == INST_READ)
            port->setPacketTimeout((uint16_t)(txpacket[PKT_PARAMETER0 + 1] + 6));
        else
            port->setPacketTimeout((uint16_t)6);  // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        // rx packet
        do {
            result = rxPacket(rxpacket);
        } while (result == COMM_SUCCESS && txpacket[PKT_ID] != rxpacket[PKT_ID]);

        if (result == COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID]) {
            if (error != 0)
                *error = (uint8_t)rxpacket[PKT_ERROR];
        }

        return result;
    }

    int ping(uint8_t id, uint16_t *model_number, uint8_t *error = 0) {
        (void)model_number;

        int result = COMM_TX_FAIL;

        uint8_t txpacket[6] = {0};
        uint8_t rxpacket[6] = {0};

        if (id >= BROADCAST_ID)
            return COMM_NOT_AVAILABLE;

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = 2;
        txpacket[PKT_INSTRUCTION] = INST_PING;

        result = txRxPacket(txpacket, rxpacket, error);
        // no model number in Protocol 1.0

        return result;
    }

    int broadcastPing(Vec<uint8_t> &id_list) {
        (void)id_list;
        return COMM_NOT_AVAILABLE;
    }

    int action(uint8_t id) {
        uint8_t txpacket[6] = {0};
        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = 2;
        txpacket[PKT_INSTRUCTION] = INST_ACTION;
        return txRxPacket(txpacket, 0);
    }

    int reboot(uint8_t id, uint8_t *error = 0) {
        (void)id;
        (void)error;
        return COMM_NOT_AVAILABLE;
    }

    int clearMultiTurn(uint8_t id, uint8_t *error = 0) {
        (void)id;
        (void)error;
        return COMM_NOT_AVAILABLE;
    }

    int factoryReset(uint8_t id, uint8_t option, uint8_t *error = 0) {
        (void)option;
        uint8_t txpacket[6] = {0};
        uint8_t rxpacket[6] = {0};
        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = 2;
        txpacket[PKT_INSTRUCTION] = INST_FACTORY_RESET;
        return txRxPacket(txpacket, rxpacket, error);
    }

    int readRx(uint8_t id, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t *rxpacket = (uint8_t *)malloc(RXPACKET_MAX_LEN);  //(length+6);

        do {
            result = rxPacket(rxpacket);
        } while (result == COMM_SUCCESS && rxpacket[PKT_ID] != id);

        if (result == COMM_SUCCESS && rxpacket[PKT_ID] == id) {
            if (error != 0)
                *error = (uint8_t)rxpacket[PKT_ERROR];

            for (uint16_t s = 0; s < length; s++) {
                data[s] = rxpacket[PKT_PARAMETER0 + s];
            }
        }

        free(rxpacket);
        return result;
    }

    int readTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t txpacket[8] = {0};
        uint8_t *rxpacket = (uint8_t *)malloc(RXPACKET_MAX_LEN);  //(length+6);

        if (id >= BROADCAST_ID) return COMM_NOT_AVAILABLE;

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = 4;
        txpacket[PKT_INSTRUCTION] = INST_READ;
        txpacket[PKT_PARAMETER0 + 0] = (uint8_t)address;
        txpacket[PKT_PARAMETER0 + 1] = (uint8_t)length;

        result = txRxPacket(txpacket, rxpacket, error);
        if (result == COMM_SUCCESS) {
            if (error != 0)
                *error = (uint8_t)rxpacket[PKT_ERROR];
            for (uint16_t s = 0; s < length; s++)
                data[s] = rxpacket[PKT_PARAMETER0 + s];
        }

        free(rxpacket);
        return result;
    }

    int writeTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;

        uint8_t *txpacket = (uint8_t *)malloc(length + 7);  //#6->7
        uint8_t rxpacket[6] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = length + 3;
        txpacket[PKT_INSTRUCTION] = INST_WRITE;
        txpacket[PKT_PARAMETER0] = (uint8_t)address;

        for (uint16_t s = 0; s < length; s++)
            txpacket[PKT_PARAMETER0 + 1 + s] = data[s];

        result = txRxPacket(txpacket, rxpacket, error);

        free(txpacket);
        return result;
    }

    int regWriteTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;

        uint8_t *txpacket = (uint8_t *)malloc(length + 6);
        uint8_t rxpacket[6] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH] = length + 3;
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE;
        txpacket[PKT_PARAMETER0] = (uint8_t)address;

        for (uint16_t s = 0; s < length; s++)
            txpacket[PKT_PARAMETER0 + 1 + s] = data[s];

        result = txRxPacket(txpacket, rxpacket, error);

        free(txpacket);
        return result;
    }
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_ */
