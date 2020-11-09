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
/// @file The file for port control in Arduino
/// @author Cho (Hancheol Cho), Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ARDUINO_PORTHANDLERARDUINO_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ARDUINO_PORTHANDLERARDUINO_H_

#include <Arduino.h>
#include "types.h"

namespace dynamixel {

class PortHandler {
private:
    const uint8_t pin_rts_enable;
    const uint8_t pin_rx_enable;
    const uint8_t pin_tx_enable;
    const size_t LATENCY_TIMER = 4;
    const size_t DEFAULT_BAUDRATE = 57600;

    int baudrate_;
    double packet_start_time_;
    double packet_timeout_;
    double tx_time_per_byte;

    Stream* stream;

public:
    PortHandler(uint8_t pin_rts_enable)
        : pin_rts_enable(pin_rts_enable), pin_rx_enable(0xFF), pin_tx_enable(0xFF), baudrate_(DEFAULT_BAUDRATE), packet_start_time_(0.0), packet_timeout_(0.0), tx_time_per_byte(0.0) {
        pinMode(pin_rts_enable, OUTPUT);
        // TODO: LOW-ACTIVE, HIGH-ACTIVE
        digitalWrite(pin_rts_enable, LOW);
    }
    PortHandler(uint8_t pin_rx_enable, uint8_t pin_tx_enable)
        : pin_rts_enable(0xFF), pin_rx_enable(pin_rx_enable), pin_tx_enable(pin_tx_enable), baudrate_(DEFAULT_BAUDRATE), packet_start_time_(0.0), packet_timeout_(0.0), tx_time_per_byte(0.0) {
        pinMode(pin_rx_enable, OUTPUT);
        pinMode(pin_tx_enable, OUTPUT);
        // TODO: LOW-ACTIVE, HIGH-ACTIVE
        digitalWrite(pin_rx_enable, LOW);
        digitalWrite(pin_tx_enable, LOW);
    }

    void attach(Stream& s, size_t baud) {
        stream = &s;
        baudrate_ = baud;
        tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
        set_tx(false);
    }

    void clearPort() {
        stream->flush();
        while (stream->available()) stream->read();
    }

    int getBytesAvailable() { return stream->available(); }

    int readPort(uint8_t* packet, int length) {
        int rx_length = stream->available();
        if (rx_length > length) rx_length = length;

        for (int i = 0; i < rx_length; i++) packet[i] = stream->read();
        return rx_length;
    }

    int writePort(uint8_t* packet, int length) {
        set_tx(true);
        int length_written = stream->write(packet, length);
        set_tx(false);
        return length_written;
    }

    void setPacketTimeout(uint16_t packet_length) {
        packet_start_time_ = getCurrentTime();
        packet_timeout_ = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
    }

    void setPacketTimeout(double msec) {
        packet_start_time_ = getCurrentTime();
        packet_timeout_ = msec;
    }

    bool isPacketTimeout() {
        if (getTimeSinceStart() > packet_timeout_) {
            packet_timeout_ = 0;
            return true;
        }
        return false;
    }

    double getCurrentTime() {
        return (double)millis();
    }

    double getTimeSinceStart() {
        double elapsed_time;

        elapsed_time = getCurrentTime() - packet_start_time_;
        if (elapsed_time < 0.0) packet_start_time_ = getCurrentTime();
        return elapsed_time;
    }

private:
    inline bool isOneRtsPin() { return !(pin_rts_enable == 0xFF); }

    void set_tx(bool b) {
        if (!b) stream->flush();  // make sure it completes before we disable...
        drv_dxl_tx_enable(b);
    }

    void drv_dxl_tx_enable(bool enable) {
        // TODO: LOW-ACTIVE, HIGH-ACTIVE
        if (isOneRtsPin()) {
            if (enable)
                digitalWrite(pin_rts_enable, HIGH);
            else
                digitalWrite(pin_rts_enable, LOW);
        } else {
            if (enable) {
                digitalWrite(pin_rx_enable, LOW);
                digitalWrite(pin_tx_enable, HIGH);
            } else {
                digitalWrite(pin_rx_enable, HIGH);
                digitalWrite(pin_tx_enable, LOW);
            }
        }
    }
};

}  // namespace dynamixel

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ARDUINO_PORTHANDLERARDUINO_H_ */
