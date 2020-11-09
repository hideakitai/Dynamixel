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
/// @file The file for Dynamixel Bulk Write
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_

#include "../../../util/ArxTypeTraits/ArxTypeTraits.h"
#include "../../../util/ArxContainer/ArxContainer.h"
#include "types.h"
// #include "port_handler.h"
#include "port_handler_arduino.h"
#include "packet_handler.h"

namespace dynamixel {

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for writing multiple Dynamixel data from different addresses with different lengths at once
////////////////////////////////////////////////////////////////////////////////
class GroupBulkWrite {
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
private:
    PortHandler *port_;
    PacketHandler *ph_;

    Vec<uint8_t> id_list_;
    Map<uint8_t, uint16_t> address_list_;  // <id, start_address>
    Map<uint8_t, uint16_t> length_list_;   // <id, data_length>
    Map<uint8_t, uint8_t *> data_list_;    // <id, data>

    bool is_param_changed_;

    uint8_t *param_;
    uint16_t param_length_;

    void makeParam() {
        if (param_ != 0)
            delete[] param_;
        param_ = 0;

        param_length_ = 0;
        for (unsigned int i = 0; i < id_list_.size(); i++)
            param_length_ += 1 + 2 + 2 + length_list_[id_list_[i]];

        param_ = new uint8_t[param_length_];

        int idx = 0;
        for (unsigned int i = 0; i < id_list_.size(); i++) {
            uint8_t id = id_list_[i];
            if (data_list_[id] == 0)
                return;

            param_[idx++] = id;
            param_[idx++] = DXL_LOBYTE(address_list_[id]);
            param_[idx++] = DXL_HIBYTE(address_list_[id]);
            param_[idx++] = DXL_LOBYTE(length_list_[id]);
            param_[idx++] = DXL_HIBYTE(length_list_[id]);
            for (uint16_t c = 0; c < length_list_[id]; c++)
                param_[idx++] = (data_list_[id])[c];
        }
    }

public:
    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that Initializes instance for Bulk Write
    /// @param port PortHandler instance
    /// @param ph PacketHandler instance
    ////////////////////////////////////////////////////////////////////////////////
    GroupBulkWrite(PortHandler *port, PacketHandler *ph)
        : port_(port),
          ph_(ph),
          is_param_changed_(false),
          param_(0),
          param_length_(0) {
        clearParam();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that calls clearParam function to clear the parameter list for Bulk Write
    ////////////////////////////////////////////////////////////////////////////////
    ~GroupBulkWrite() { clearParam(); }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that returns PortHandler instance
    /// @return PortHandler instance
    ////////////////////////////////////////////////////////////////////////////////
    PortHandler *getPortHandler() { return port_; }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that returns PacketHandler instance
    /// @return PacketHandler instance
    ////////////////////////////////////////////////////////////////////////////////
    PacketHandler *getPacketHandler() { return ph_; }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that adds id, start_address, data_length to the Bulk Write list
    /// @param id Dynamixel ID
    /// @param start_address Address of the data for write
    /// @param data_length Length of the data for write
    /// @return false
    /// @return   when the ID exists already in the list
    /// @return or true
    ////////////////////////////////////////////////////////////////////////////////
    bool addParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data) {
        // if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        //   return false;
        // if id is already exist, return false
        for (const auto &i : id_list_)  // if id is already exist
            if (i == id) return false;

        id_list_.push_back(id);
        address_list_[id] = start_address;
        length_list_[id] = data_length;
        data_list_[id] = new uint8_t[data_length];
        for (uint16_t c = 0; c < data_length; c++)
            data_list_[id][c] = data[c];

        is_param_changed_ = true;
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that removes id from the Bulk Write list
    /// @param id Dynamixel ID
    ////////////////////////////////////////////////////////////////////////////////
    void removeParam(uint8_t id) {
        // if id is NOT exist, return
        Vec<uint8_t>::iterator it = id_list_.begin();
        for (; it != id_list_.end(); ++it)
            if (*it == id) break;
        if (it == id_list_.end()) return;

        id_list_.erase(it);
        address_list_.erase(id);
        length_list_.erase(id);
        delete[] data_list_[id];
        data_list_.erase(id);

        is_param_changed_ = true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that changes the data for write in id -> start_address -> data_length to the Bulk Write list
    /// @param id Dynamixel ID
    /// @param start_address Address of the data for write
    /// @param data_length Length of the data for write
    /// @param data for replacement
    /// @return false
    /// @return   when the ID doesn't exist in the list
    /// @return or true
    ////////////////////////////////////////////////////////////////////////////////
    bool changeParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data) {
        // if id is NOT exist, return false
        Vec<uint8_t>::iterator it = id_list_.begin();
        for (; it != id_list_.end(); ++it)
            if (*it == id) break;
        if (it == id_list_.end()) return false;

        address_list_[id] = start_address;
        length_list_[id] = data_length;
        delete[] data_list_[id];
        data_list_[id] = new uint8_t[data_length];
        for (uint16_t c = 0; c < data_length; c++)
            data_list_[id][c] = data[c];

        is_param_changed_ = true;
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that clears the Bulk Write list
    ////////////////////////////////////////////////////////////////////////////////
    void clearParam() {
        for (unsigned int i = 0; i < id_list_.size(); i++)
            delete[] data_list_[id_list_[i]];

        id_list_.clear();
        address_list_.clear();
        length_list_.clear();
        data_list_.clear();
        if (param_ != 0)
            delete[] param_;
        param_ = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that transmits the Bulk Write instruction packet which might be constructed by GroupBulkWrite::addParam function
    /// @return COMM_NOT_AVAILABLE
    /// @return   when the list for Bulk Write is empty
    /// @return   when Protocol1.0 has been used
    /// @return or the other communication results which come from PacketHandler::bulkWriteTxOnly
    ////////////////////////////////////////////////////////////////////////////////
    int txPacket() {
        if (is_param_changed_ == true || param_ == 0)
            makeParam();

        return bulkWriteTxOnly(port_, param_, param_length_);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that transmits INST_BULK_WRITE instruction packet
    /// @description The function makes an instruction packet with INST_BULK_WRITE,
    /// @description transmits the packet with Protocol2PacketHandler::txRxPacket().
    /// @param port PortHandler instance
    /// @param param Parameter for Bulk Write {ID1, START_ADDR_L, START_ADDR_H, DATA_LEN_L, DATA_LEN_H, DATA0, DATA1, ..., DATAn, ID2, START_ADDR_L, START_ADDR_H, DATA_LEN_L, DATA_LEN_H, DATA0, DATA1, ..., DATAn}
    /// @param param_length Length of the data for Bulk Write
    /// @return communication results which come from Protocol2PacketHandler::txRxPacket()
    ////////////////////////////////////////////////////////////////////////////////
    int bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length) {
        (void)port;
        int result = COMM_TX_FAIL;

        uint8_t *txpacket = (uint8_t *)malloc(param_length + 10 + (param_length / 3));
        // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

        if (txpacket == NULL)
            return result;

        txpacket[PKT_ID] = BROADCAST_ID;
        txpacket[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 3);  // 3: INST CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 3);  // 3: INST CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION] = INST_BULK_WRITE;

        for (uint16_t s = 0; s < param_length; s++)
            txpacket[PKT_PARAMETER0 + s] = param[s];
        //memcpy(&txpacket[PKT_PARAMETER0], param, param_length);

        result = ph_->txRxPacket(txpacket, 0, 0);

        free(txpacket);
        //delete[] txpacket;
        return result;
    }
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_ */
