/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Bulk Read and Bulk Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
//

#include <DynamixelSDK.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE     -150000              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define CMD_SERIAL                      Serial


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
    dx.addModel<arduino::Model::X>(1);
    dx.addModel<arduino::Model::MX>(2);

    delay(2000);

    dx.power(1, true);
    dx.power(2, true);

    dx.add_bulk_read_target(1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    dx.add_bulk_read_target(2, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
}

void loop()
{
    static bool dir = true;
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
    uint8_t param_goal_position[4];

    if (dir)
    {
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[(int)dir]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[(int)dir]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[(int)dir]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[(int)dir]));

        uint8_t led_value = 0xFF;

        dx.add_bulk_target(1, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
        dx.add_bulk_target(2, ADDR_PRO_LED_RED, LEN_PRO_RED_LED, param_goal_position, &led_value);
    }
    else
    {
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[(int)dir]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[(int)dir]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[(int)dir]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[(int)dir]));

        uint8_t led_value = 0x00;

        dx.add_bulk_target(1, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
        dx.add_bulk_target(2, ADDR_PRO_LED_RED, LEN_PRO_RED_LED, param_goal_position, &led_value);
    }

    dx.bulk_write();

    delay(500);

    dx.bulk_read_request();

    if (dx.available(1))
    {
        Serial.print("ID 1 current position = ");
        Serial.println(dx.bulk_read_data(1));
    }
    if (dx.available(2))
    {
        Serial.print("ID 2 LED data = ");
        Serial.println(dx.bulk_read_data(2));
    }

    dir = !dir;
}
