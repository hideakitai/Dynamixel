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
/// @file The file for Dynamixel Sync Read
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_

#include "../../../util/ArxTypeTraits/ArxTypeTraits.h"
#include "../../../util/ArxContainer/ArxContainer.h"
#include "types.h"
#include "port_handler_arduino.h"
#include "packet_handler.h"

namespace dynamixel {

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for reading multiple Dynamixel data from same address with same length at once
////////////////////////////////////////////////////////////////////////////////
class GroupSyncRead {
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
    Map<uint8_t, uint8_t *> data_list_;   // <id, data>
    Map<uint8_t, uint8_t *> error_list_;  // <id, error>

    bool last_result_;
    bool is_param_changed_;

    uint8_t *param_;
    uint16_t start_address_;
    uint16_t data_length_;

    void makeParam() {
        if (param_ != 0)
            delete[] param_;
        param_ = 0;

        param_ = new uint8_t[id_list_.size() * 1];  // ID(1)

        int idx = 0;
        for (unsigned int i = 0; i < id_list_.size(); i++)
            param_[idx++] = id_list_[i];
    }

public:
    void setAddress(uint16_t addr, uint16_t size) {
        start_address_ = addr;
        data_length_ = size;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that Initializes instance for Sync Read
    /// @param port PortHandler instance
    /// @param ph PacketHandler instance
    /// @param start_address Address of the data for read
    /// @param data_length Length of the data for read
    ////////////////////////////////////////////////////////////////////////////////
    //   GroupSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);
    GroupSyncRead(PortHandler *port, PacketHandler *ph)
        : port_(port),
          ph_(ph),
          last_result_(false),
          is_param_changed_(false),
          param_(0)
    // start_address_(start_address),
    // data_length_(data_length)
    {
        clearParam();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that calls clearParam function to clear the parameter list for Sync Read
    ////////////////////////////////////////////////////////////////////////////////
    ~GroupSyncRead() { clearParam(); }

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
    /// @brief The function that adds id, start_address, data_length to the Sync Read list
    /// @param id Dynamixel ID
    /// @return false
    /// @return   when the ID exists already in the list
    /// @return   when the protocol1.0 has been used
    /// @return or true
    ////////////////////////////////////////////////////////////////////////////////
    bool addParam(uint8_t id) {
        // if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        //   return false;
        // if id is already exist, return false
        for (const auto &i : id_list_)  // if id is already exist
            if (i == id) return false;

        id_list_.push_back(id);
        data_list_[id] = new uint8_t[data_length_];
        error_list_[id] = new uint8_t[1];

        is_param_changed_ = true;
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that removes id from the Sync Read list
    /// @param id Dynamixel ID
    ////////////////////////////////////////////////////////////////////////////////
    void removeParam(uint8_t id) {
        // if id is NOT exist, return
        Vec<uint8_t>::iterator it = id_list_.begin();
        for (; it != id_list_.end(); ++it)
            if (*it == id) break;
        if (it == id_list_.end()) return;

        id_list_.erase(it);
        delete[] data_list_[id];
        delete[] error_list_[id];
        data_list_.erase(id);
        error_list_.erase(id);

        is_param_changed_ = true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that clears the Sync Read list
    ////////////////////////////////////////////////////////////////////////////////
    void clearParam() {
        for (unsigned int i = 0; i < id_list_.size(); i++) {
            delete[] data_list_[id_list_[i]];
            delete[] error_list_[id_list_[i]];
        }

        id_list_.clear();
        data_list_.clear();
        error_list_.clear();
        if (param_ != 0)
            delete[] param_;
        param_ = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that transmits the Sync Read instruction packet which might be constructed by GroupSyncRead::addParam function
    /// @return COMM_NOT_AVAILABLE
    /// @return   when the list for Sync Read is empty
    /// @return   when the protocol1.0 has been used
    /// @return or the other communication results which come from PacketHandler::syncReadTx
    ////////////////////////////////////////////////////////////////////////////////
    int txPacket() {
        if (is_param_changed_ == true || param_ == 0)
            makeParam();

        return syncReadTx(port_, start_address_, data_length_, param_, (uint16_t)id_list_.size() * 1);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that receives the packet which might be come from the Dynamixel
    /// @return COMM_NOT_AVAILABLE
    /// @return   when the list for Sync Read is empty
    /// @return   when the protocol1.0 has been used
    /// @return COMM_SUCCESS
    /// @return   when there is packet recieved
    /// @return or the other communication results
    ////////////////////////////////////////////////////////////////////////////////
    int rxPacket() {
        last_result_ = false;

        int cnt = id_list_.size();
        int result = COMM_RX_FAIL;

        if (cnt == 0)
            return COMM_NOT_AVAILABLE;

        for (int i = 0; i < cnt; i++) {
            uint8_t id = id_list_[i];

            result = ph_->readRx(id, data_length_, data_list_[id], error_list_[id]);
            if (result != COMM_SUCCESS)
                return result;
        }

        if (result == COMM_SUCCESS)
            last_result_ = true;

        return result;
    }
    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that transmits and receives the packet which might be come from the Dynamixel
    /// @return COMM_NOT_AVAILABLE
    /// @return   when the protocol1.0 has been used
    /// @return COMM_RX_FAIL
    /// @return   when there is no packet recieved
    /// @return COMM_SUCCESS
    /// @return   when there is packet recieved
    /// @return or the other communication results which come from GroupBulkRead::txPacket or GroupBulkRead::rxPacket
    ////////////////////////////////////////////////////////////////////////////////
    int txRxPacket() {
        int result = COMM_TX_FAIL;

        result = txPacket();
        if (result != COMM_SUCCESS)
            return result;

        return rxPacket();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that checks whether there are available data which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
    /// @param id Dynamixel ID
    /// @param address Address of the data for read
    /// @param data_length Length of the data for read
    /// @return false
    /// @return   when there are no data available
    /// @return   when the protocol1.0 has been used
    /// @return or true
    ////////////////////////////////////////////////////////////////////////////////
    //   bool        isAvailable (uint8_t id, uint16_t address, uint16_t data_length);
    bool isAvailable(uint8_t id) {
        if (last_result_ == false || data_list_.find(id) == data_list_.end())
            return false;

        // TODO: ?????
        //   if (address < start_address_ || start_address_ + data_length_ - data_length < address)
        //     return false;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that gets the data which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
    /// @param id Dynamixel ID
    /// @param address Address of the data for read
    /// @data_length Length of the data for read
    /// @return data value
    ////////////////////////////////////////////////////////////////////////////////
    //   uint32_t    getData     (uint8_t id, uint16_t address, uint16_t data_length);
    uint32_t getData(uint8_t id) {
        switch (data_length_) {
            case 1:
                //   return data_list_[id][address - start_address_];
                return data_list_[id][0];

            case 2:
                //   return DXL_MAKEWORD(data_list_[id][address - start_address_], data_list_[id][address - start_address_ + 1]);
                return DXL_MAKEWORD(data_list_[id][0], data_list_[id][1]);

            case 4:
                //   return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - start_address_ + 0], data_list_[id][address - start_address_ + 1]),
                //              DXL_MAKEWORD(data_list_[id][address - start_address_ + 2], data_list_[id][address - start_address_ + 3]));
                return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][0], data_list_[id][1]), DXL_MAKEWORD(data_list_[id][2], data_list_[id][3]));

            default:
                return 0;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that gets the error which might be received by GroupSyncRead::rxPacket or GroupSyncRead::txRxPacket
    /// @param id Dynamixel ID
    /// @error error of Dynamixel
    /// @return true
    /// @return   when Dynamixel returned specific error byte
    /// @return or false
    ////////////////////////////////////////////////////////////////////////////////
    bool getError(uint8_t id, uint8_t *error) {
        // TODO : check protocol version, last_result_, data_list
        // if (last_result_ == false || error_list_.find(id) == error_list_.end())

        error[0] = error_list_[id][0];

        if (error[0] != 0) {
            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief The function that transmits INST_SYNC_READ instruction packet
    /// @description The function makes an instruction packet with INST_SYNC_READ,
    /// @description transmits the packet with Protocol2PacketHandler::txPacket().
    /// @param port PortHandler instance
    /// @param start_address Address of the data for Sync Read
    /// @param data_length Length of the data for Sync Read
    /// @param param Parameter for Sync Read
    /// @param param_length Length of the data for Sync Read
    /// @return communication results which come from Protocol2PacketHandler::txPacket()
    ////////////////////////////////////////////////////////////////////////////////
    int syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
    // SyncReadRx   -> GroupSyncRead class
    // SyncReadTxRx -> GroupSyncRead class
    {
        int result = COMM_TX_FAIL;

        uint8_t *txpacket = (uint8_t *)malloc(param_length + 14 + (param_length / 3));
        // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

        if (txpacket == NULL)
            return result;

        txpacket[PKT_ID] = BROADCAST_ID;
        txpacket[PKT_LENGTH_L] = DXL_LOBYTE(param_length + 7);  // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H] = DXL_HIBYTE(param_length + 7);  // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION] = INST_SYNC_READ;
        txpacket[PKT_PARAMETER0 + 0] = DXL_LOBYTE(start_address);
        txpacket[PKT_PARAMETER0 + 1] = DXL_HIBYTE(start_address);
        txpacket[PKT_PARAMETER0 + 2] = DXL_LOBYTE(data_length);
        txpacket[PKT_PARAMETER0 + 3] = DXL_HIBYTE(data_length);

        for (uint16_t s = 0; s < param_length; s++)
            txpacket[PKT_PARAMETER0 + 4 + s] = param[s];
        //memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

        result = ph_->txPacket(txpacket);
        if (result == COMM_SUCCESS)
            port->setPacketTimeout((uint16_t)((11 + data_length) * param_length));

        free(txpacket);
        return result;
    }
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_ */
