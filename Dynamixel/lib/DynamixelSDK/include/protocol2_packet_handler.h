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
/// @file The file for Protocol 2.0 Dynamixel packet control
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_H_

#include "types.h"
#include "packet_handler.h"

namespace dynamixel {

#ifdef __AVR__
static constexpr uint16_t crc_table[256] PROGMEM = {
#else
static constexpr uint16_t crc_table[256] = {
#endif
    0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202};

inline uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    uint16_t i = 0;
    for (uint16_t j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
#ifdef __AVR__
        crc_accum = (crc_accum << 8) ^ pgm_read_byte_near(crc_table + i);
#else
        crc_accum = (crc_accum << 8) ^ crc_table[i];
#endif
    }

    return crc_accum;
}

class Protocol2PacketHandler : public PacketHandler {
    static const size_t TXPACKET_MAX_LEN = 1 * 1024;
    static const size_t RXPACKET_MAX_LEN = 1 * 1024;

    ///////////////// for Protocol 2.0 Packet /////////////////
    static const uint8_t PKT_HEADER0 = 0;
    static const uint8_t PKT_HEADER1 = 1;
    static const uint8_t PKT_HEADER2 = 2;
    static const uint8_t PKT_RESERVED = 3;
    static const uint8_t PKT_ID = 4;
    static const uint8_t PKT_LENGTH_L = 5;
    static const uint8_t PKT_LENGTH_H = 6;
    static const uint8_t PKT_INSTRUCTION = 7;
    static const uint8_t PKT_ERROR = 8;
    static const uint8_t PKT_PARAMETER0 = 8;

    ///////////////// Protocol 2.0 Error bit /////////////////
    static const uint8_t ERRNUM_RESULT_FAIL = 1;  // Failed to process the instruction packet.
    static const uint8_t ERRNUM_INSTRUCTION = 2;  // Instruction error
    static const uint8_t ERRNUM_CRC = 3;          // CRC check error
    static const uint8_t ERRNUM_DATA_RANGE = 4;   // Data range error
    static const uint8_t ERRNUM_DATA_LENGTH = 5;  // Data length error
    static const uint8_t ERRNUM_DATA_LIMIT = 6;   // Data limit error
    static const uint8_t ERRNUM_ACCESS = 7;       // Access error
    static const uint8_t ERRBIT_ALERT = 128;      //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

    void addStuffing(uint8_t *packet) {
        int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
        int packet_length_out = packet_length_in;

        if (packet_length_in < 8)  // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
            return;

        uint8_t *packet_ptr;
        uint16_t packet_length_before_crc = packet_length_in - 2;
        for (uint16_t i = 3; i < packet_length_before_crc; i++) {
            packet_ptr = &packet[i + PKT_INSTRUCTION - 2];
            if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
                packet_length_out++;
        }

        if (packet_length_in == packet_length_out)  // no stuffing required
            return;

        uint16_t out_index = packet_length_out + 6 - 2;  // last index before crc
        uint16_t in_index = packet_length_in + 6 - 2;    // last index before crc
        while (out_index != in_index) {
            if (packet[in_index] == 0xFD && packet[in_index - 1] == 0xFF && packet[in_index - 2] == 0xFF) {
                packet[out_index--] = 0xFD;  // byte stuffing
                if (out_index != in_index) {
                    packet[out_index--] = packet[in_index--];  // FD
                    packet[out_index--] = packet[in_index--];  // FF
                    packet[out_index--] = packet[in_index--];  // FF
                }
            } else {
                packet[out_index--] = packet[in_index--];
            }
        }

        packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
        packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

        return;
    }

    void removeStuffing(uint8_t *packet) {
        int i = 0, index = 0;
        int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
        int packet_length_out = packet_length_in;

        index = PKT_INSTRUCTION;
        for (i = 0; i < packet_length_in - 2; i++)  // except CRC
        {
            if (packet[i + PKT_INSTRUCTION] == 0xFD && packet[i + PKT_INSTRUCTION + 1] == 0xFD && packet[i + PKT_INSTRUCTION - 1] == 0xFF && packet[i + PKT_INSTRUCTION - 2] == 0xFF) {  // FF FF FD FD
                packet_length_out--;
                i++;
            }
            packet[index++] = packet[i + PKT_INSTRUCTION];
        }
        packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 2];
        packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 1];

        packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
        packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
    }

public:
    ~Protocol2PacketHandler() {}

    float getProtocolVersion() { return 2.0; }

    const char *getRxPacketError(uint8_t error) {
#ifndef __AVR__
        if (error & ERRBIT_ALERT)
            return "[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!";

        const uint8_t not_alert_error = error & ~ERRBIT_ALERT;

        switch (not_alert_error) {
            case 0:
                return "";
            case ERRNUM_RESULT_FAIL:
                return "[RxPacketError] Failed to process the instruction packet!";
            case ERRNUM_INSTRUCTION:
                return "[RxPacketError] Undefined instruction or incorrect instruction!";
            case ERRNUM_CRC:
                return "[RxPacketError] CRC doesn't match!";
            case ERRNUM_DATA_RANGE:
                return "[RxPacketError] The data value is out of range!";
            case ERRNUM_DATA_LENGTH:
                return "[RxPacketError] The data length does not match as expected!";
            case ERRNUM_DATA_LIMIT:
                return "[RxPacketError] The data value exceeds the limit value!";
            case ERRNUM_ACCESS:
                return "[RxPacketError] Writing or Reading is not available to target address!";
            default:
                return "[RxPacketError] Unknown error code!";
        }
#else
        (void)error;
        return "";
#endif
    }

    int txPacket(uint8_t *txpacket) {
        uint16_t total_packet_length = 0;
        uint16_t written_packet_length = 0;

        addStuffing(txpacket);

        total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;
        // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
        if (total_packet_length > TXPACKET_MAX_LEN)
            return COMM_TX_ERROR;

        txpacket[PKT_HEADER0] = 0xFF;
        txpacket[PKT_HEADER1] = 0xFF;
        txpacket[PKT_HEADER2] = 0xFD;
        txpacket[PKT_RESERVED] = 0x00;

        uint16_t crc = updateCRC(0, txpacket, total_packet_length - 2);  // 2: CRC16
        txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
        txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

        // tx packet
        port->clearPort();
        written_packet_length = port->writePort(txpacket, total_packet_length);
        if (total_packet_length != written_packet_length)
            return COMM_TX_FAIL;

        return COMM_SUCCESS;
    }

    int rxPacket(uint8_t *rxpacket) {
        int result = COMM_TX_FAIL;

        uint16_t rx_length = 0;
        uint16_t wait_length = 11;  // minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H)

        while (true) {
            rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
            if (rx_length >= wait_length) {
                uint16_t idx = 0;

                // find packet header
                for (idx = 0; idx < (rx_length - 3); idx++) {
                    if ((rxpacket[idx] == 0xFF) && (rxpacket[idx + 1] == 0xFF) && (rxpacket[idx + 2] == 0xFD) && (rxpacket[idx + 3] != 0xFD))
                        break;
                }

                if (idx == 0)  // found at the beginning of the packet
                {
                    if (rxpacket[PKT_RESERVED] != 0x00 ||
                        rxpacket[PKT_ID] > 0xFC ||
                        DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN ||
                        rxpacket[PKT_INSTRUCTION] != 0x55) {
                        // remove the first byte in the packet
                        for (uint16_t s = 0; s < rx_length - 1; s++)
                            rxpacket[s] = rxpacket[1 + s];
                        rx_length -= 1;
                        continue;
                    }

                    // re-calculate the exact length of the rx packet
                    if (wait_length != DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1) {
                        wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
                        continue;
                    }

                    if (rx_length < wait_length) {
                        if (port->isPacketTimeout() == true) {
                            if (rx_length == 0)
                                result = COMM_RX_TIMEOUT;
                            else
                                result = COMM_RX_CORRUPT;
                            break;
                        } else
                            continue;
                    }

                    uint16_t crc = DXL_MAKEWORD(rxpacket[wait_length - 2], rxpacket[wait_length - 1]);
                    if (updateCRC(0, rxpacket, wait_length - 2) == crc)
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

        if (result == COMM_SUCCESS) removeStuffing(rxpacket);

        return result;
    }

    int txRxPacket(uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;

        result = txPacket(txpacket);
        if (result != COMM_SUCCESS) return result;

        // (Instruction == BulkRead or SyncRead) == this function is not available.
        if (txpacket[PKT_INSTRUCTION] == INST_BULK_READ || txpacket[PKT_INSTRUCTION] == INST_SYNC_READ)
            result = COMM_NOT_AVAILABLE;

        // (ID == Broadcast ID) == no need to wait for status packet or not available.
        // (Instruction == action) == no need to wait for status packet
        if (txpacket[PKT_ID] == BROADCAST_ID || txpacket[PKT_INSTRUCTION] == INST_ACTION)
            return result;

        // set packet timeout
        if (txpacket[PKT_INSTRUCTION] == INST_READ)
            port->setPacketTimeout((uint16_t)(DXL_MAKEWORD(txpacket[PKT_PARAMETER0 + 2], txpacket[PKT_PARAMETER0 + 3]) + 11));
        else
            port->setPacketTimeout((uint16_t)11);
        // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H

        // rx packet
        do {
            result = rxPacket(rxpacket);
        } while (result == COMM_SUCCESS && txpacket[PKT_ID] != rxpacket[PKT_ID]);

        if (result == COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID])
            if (error != 0) *error = (uint8_t)rxpacket[PKT_ERROR];

        return result;
    }

    int ping(uint8_t id, uint16_t *model_number, uint8_t *error = 0) {
        if (id >= BROADCAST_ID) return COMM_NOT_AVAILABLE;

        uint8_t txpacket[10] = {0};
        uint8_t rxpacket[14] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 3;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_PING;

        int result = txRxPacket(txpacket, rxpacket, error);
        if (result == COMM_SUCCESS && model_number != 0)
            *model_number = DXL_MAKEWORD(rxpacket[PKT_PARAMETER0 + 1], rxpacket[PKT_PARAMETER0 + 2]);

        return result;
    }

    int broadcastPing(Vec<uint8_t> &id_list) {
        const int STATUS_LENGTH = 14;
        int result = COMM_TX_FAIL;

        id_list.clear();

        uint16_t rx_length = 0;
        uint16_t wait_length = STATUS_LENGTH * MAX_ID;

        uint8_t txpacket[10] = {0};
        uint8_t rxpacket[STATUS_LENGTH * MAX_ID] = {0};

        txpacket[PKT_ID] = BROADCAST_ID;
        txpacket[PKT_LENGTH_L] = 3;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_PING;

        result = txPacket(txpacket);
        if (result != COMM_SUCCESS) return result;

        // set rx timeout
        port->setPacketTimeout((uint16_t)(wait_length * 30));

        while (1) {
            rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
            if (port->isPacketTimeout() == true)  // || rx_length >= wait_length)
                break;
        }

        if (rx_length == 0) return COMM_RX_TIMEOUT;

        while (1) {
            if (rx_length < STATUS_LENGTH)
                return COMM_RX_CORRUPT;

            uint16_t idx = 0;

            // find packet header
            for (idx = 0; idx < (rx_length - 2); idx++) {
                if (rxpacket[idx] == 0xFF && rxpacket[idx + 1] == 0xFF && rxpacket[idx + 2] == 0xFD)
                    break;
            }

            if (idx == 0)  // found at the beginning of the packet
            {
                uint16_t crc = DXL_MAKEWORD(rxpacket[STATUS_LENGTH - 2], rxpacket[STATUS_LENGTH - 1]);

                if (updateCRC(0, rxpacket, STATUS_LENGTH - 2) == crc) {
                    result = COMM_SUCCESS;
                    id_list.push_back(rxpacket[PKT_ID]);
                    for (uint16_t s = 0; s < rx_length - STATUS_LENGTH; s++)
                        rxpacket[s] = rxpacket[STATUS_LENGTH + s];
                    rx_length -= STATUS_LENGTH;
                    if (rx_length == 0) return result;
                } else {
                    result = COMM_RX_CORRUPT;
                    // remove header (0xFF 0xFF 0xFD)
                    for (uint16_t s = 0; s < rx_length - 3; s++)
                        rxpacket[s] = rxpacket[3 + s];
                    rx_length -= 3;
                }
            } else {
                // remove unnecessary packets
                for (uint16_t s = 0; s < rx_length - idx; s++)
                    rxpacket[s] = rxpacket[idx + s];
                rx_length -= idx;
            }
        }
        return result;
    }

    int action(uint8_t id) {
        uint8_t txpacket[10] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 3;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_ACTION;

        return txRxPacket(txpacket, 0);
    }

    int reboot(uint8_t id, uint8_t *error = 0) {
        uint8_t txpacket[10] = {0};
        uint8_t rxpacket[11] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 3;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_REBOOT;

        return txRxPacket(txpacket, rxpacket, error);
    }

    int clearMultiTurn(uint8_t id, uint8_t *error = 0) {
        uint8_t txpacket[15] = {0};
        uint8_t rxpacket[11] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 8;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_CLEAR;
        txpacket[PKT_PARAMETER0] = 0x01;
        txpacket[PKT_PARAMETER0 + 1] = 0x44;
        txpacket[PKT_PARAMETER0 + 2] = 0x58;
        txpacket[PKT_PARAMETER0 + 3] = 0x4C;
        txpacket[PKT_PARAMETER0 + 4] = 0x22;

        return txRxPacket(txpacket, rxpacket, error);
    }

    int factoryReset(uint8_t id, uint8_t option, uint8_t *error = 0) {
        uint8_t txpacket[11] = {0};
        uint8_t rxpacket[11] = {0};

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 4;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_FACTORY_RESET;
        txpacket[PKT_PARAMETER0] = option;

        return txRxPacket(txpacket, rxpacket, error);
    }

    int readRx(uint8_t id, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t *rxpacket = (uint8_t *)malloc(length + 11 + (length / 3));

        do {
            result = rxPacket(rxpacket);
        } while (result == COMM_SUCCESS && rxpacket[PKT_ID] != id);

        if (result == COMM_SUCCESS && rxpacket[PKT_ID] == id) {
            if (error != 0)
                *error = (uint8_t)rxpacket[PKT_ERROR];
            for (uint16_t s = 0; s < length; s++)
                data[s] = rxpacket[PKT_PARAMETER0 + 1 + s];
        }
        free(rxpacket);
        return result;
    }

    int readTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t txpacket[14] = {0};
        uint8_t *rxpacket = (uint8_t *)malloc(length + 11 + (length / 3));

        if (rxpacket == NULL) return result;
        if (id >= BROADCAST_ID) return COMM_NOT_AVAILABLE;

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = 7;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_READ;
        txpacket[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
        txpacket[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);
        txpacket[PKT_PARAMETER0 + 2] = (uint8_t)DXL_LOBYTE(length);
        txpacket[PKT_PARAMETER0 + 3] = (uint8_t)DXL_HIBYTE(length);

        result = txRxPacket(txpacket, rxpacket, error);
        if (result == COMM_SUCCESS) {
            if (error != 0)
                *error = (uint8_t)rxpacket[PKT_ERROR];
            for (uint16_t s = 0; s < length; s++)
                data[s] = rxpacket[PKT_PARAMETER0 + 1 + s];
        }

        free(rxpacket);
        return result;
    }

    int writeTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t *txpacket = (uint8_t *)malloc(length + 12 + (length / 3));
        uint8_t rxpacket[11] = {0};

        if (txpacket == NULL) return result;

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
        txpacket[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
        txpacket[PKT_INSTRUCTION] = INST_WRITE;
        txpacket[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
        txpacket[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

        for (uint16_t s = 0; s < length; s++)
            txpacket[PKT_PARAMETER0 + 2 + s] = data[s];

        result = txRxPacket(txpacket, rxpacket, error);

        free(txpacket);
        return result;
    }

    int regWriteTxRx(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) {
        int result = COMM_TX_FAIL;
        uint8_t *txpacket = (uint8_t *)malloc(length + 12 + (length / 3));
        uint8_t rxpacket[11] = {0};

        if (txpacket == NULL) return result;

        txpacket[PKT_ID] = id;
        txpacket[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
        txpacket[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE;
        txpacket[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
        txpacket[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

        for (uint16_t s = 0; s < length; s++)
            txpacket[PKT_PARAMETER0 + 2 + s] = data[s];

        result = txRxPacket(txpacket, rxpacket, error);

        free(txpacket);
        return result;
    }
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL2PACKETHANDLER_H_ */
