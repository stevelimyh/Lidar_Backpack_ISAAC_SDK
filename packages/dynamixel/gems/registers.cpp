/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "registers.hpp"

namespace isaac {
namespace dynamixel {

std::map<RegisterKey, ServoRegister> InitRegisters(Model model) {
  // http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table
  if ((model == Model::AX12A) || (model == Model::MX12W)) {
    std::map<RegisterKey, ServoRegister> map {
      {RegisterKey::MODEL_NUMBER,     {0, 2, 0, 0}},
      {RegisterKey::FIRMWARE_VERSION, {2, 1, 0, 0}},                     //  Firmware Version
      {RegisterKey::DYNAMIXEL_ID, {3, 1, 1, 64}},                        //  DYNAMIXEL ID
      {RegisterKey::BAUDRATE, {4, 1, 1, 7}},                             //  Communication Speed
      {RegisterKey::RETURN_DELAY_TIME, {5, 1, 0, 254}},                  //  Response Delay Time
      {RegisterKey::CW_ANGLE_LIMIT, {6, 2, 0, 1023}},                    //  Clockwise Angle Limit - The unit is 0.29°.
      {RegisterKey::CCW_ANGLE_LIMIT, {8, 2, 0, 1023}},                   //  Counter-Clockwise Angle Limit - The unit is 0.29°.
      {RegisterKey::TEMPERATURE_LIMIT, {11, 1, 0, 99}},                  //  Maximum Internal Temperature Limit - Do not change!!
      {RegisterKey::MIN_VOLTAGE_LIMIT, {12, 1, 50, 160}},                //  Minimum Input Voltage Limit
      {RegisterKey::MAX_VOLTAGE_LIMIT, {13, 1, 50, 160}},                //  Maximum Input Voltage Limit
      {RegisterKey::MAX_TORQUE, {14, 2, 0, 1023}},                       //  Maximum Torque -  the unit is about 0.1%.
      {RegisterKey::STATUS_RETURN_LEVEL, {16, 1, 0, 3}},                 //  Select Types of Status Return (0 - PING, 1 PING READ or 2 ALL)
      {RegisterKey::ALARM_LED, {17, 1, 0, 3}},                           //  LED for Alarm
      {RegisterKey::SHUTDOWN, {18, 1, 0, 3}},                            //  Shutdown Error Information
      {RegisterKey::TORQUE_ENABLE, {24, 1, 0, 1}},                       //  Motor Torque On/Off
      {RegisterKey::LED, {25, 1, 0, 1}},                                 //  Status LED On/Off
      {RegisterKey::GOAL_POSITION, {30, 2, 0, 1023}},                    //  Target Position
      {RegisterKey::MOVING_SPEED, {32, 2, 0, 2047}},                     //  Moving Speed | Wheel Mode: 0 ~ 2,047(0x7FF) can be used, the unit is about 0.1%. | Join Mode: 0 ~ 1,023(0x3FF) can be used, and the unit is about 0.111rpm.
      {RegisterKey::TORQUE_LIMIT, {34, 2, 0, 1023}},                     //  Torque Limit(Goal Torque) unit is about 0.1%.
      {RegisterKey::CURRENT_POSITION, {36, 2, 0, 1023}},                 //  Present Position - unit is 0.29 [°]
      {RegisterKey::CURRENT_SPEED, {38, 2, 0, 2047}},                    //  Present Speed | Joint Mode - The unit is about 0.111rpm. | Wheel Mode - the unit is about 0.1%.
      {RegisterKey::CURRENT_LOAD, {40, 2, 0, 2047}},                     //  Present Load - unit is 0.1%.
      {RegisterKey::CURRENT_VOLTAGE, {42, 1, 0, 255}},                   //  Present Voltage - This value is 10 times larger than the actual voltage
      {RegisterKey::CURRENT_TEMPERATURE, {43, 1, 0, 255}},               //  Present Temperature in degree
      {RegisterKey::MOVING, {46, 1, 0, 1}}                               //  Movement Status
    };
    return map;
  } else if ((model == Model::XM430) || (model == Model::XC430)){
    // http://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
    // http://emanual.robotis.com/docs/en/dxl/x/xc430-w150/
    std::map<RegisterKey, ServoRegister> map {
      {RegisterKey::MODEL_NUMBER,     {0, 2, 0, 0}},
      {RegisterKey::FIRMWARE_VERSION, {6, 1, 0, 0}},                     //  Firmware Version
      {RegisterKey::DYNAMIXEL_ID, {7, 1, 1, 253}},                       //  DYNAMIXEL ID
      {RegisterKey::BAUDRATE, {8, 1, 0, 7}},                             //  Communication Speed
      {RegisterKey::RETURN_DELAY_TIME, {9, 1, 0, 254}},                  //  Response Delay Time
      {RegisterKey::DRIVE_MODE, {10, 1, 0, 7}},                          //  Drive mode
      {RegisterKey::OPERATING_MODE, {11, 1, 0, 16}},                     //  Operating mode
      {RegisterKey::PROTOCOL_VERSION, {13, 1, 1, 2}},                    //  Protocol version
      {RegisterKey::HOME_POSITION_OFFSET, {20, 4, -1044479, 1044479}},   //  Home offset
      {RegisterKey::TEMPERATURE_LIMIT, {31, 1, 0, 1023}},                //  Maximum Internal Temperature Limit - Do not change!!
      {RegisterKey::MIN_VOLTAGE_LIMIT, {34, 2, 95, 160}},                //  Minimum Input Voltage Limit
      {RegisterKey::MAX_VOLTAGE_LIMIT, {32, 2, 95, 160}},                //  Maximum Input Voltage Limit
      {RegisterKey::SHUTDOWN, {63, 1, 0, 0}},                            //  Shutdown Error Information
      {RegisterKey::LED, {65, 1, 0, 1}},                                 //  Status LED On/Off
      {RegisterKey::CURRENT_POSITION, {132, 4, 0, 0}},                   //  Present Position - unit is 0.229 [°]
      {RegisterKey::CURRENT_SPEED, {128, 4, 0, 0}},                      //  Present Speed | Joint Mode - The unit is about 0.111rpm. | Wheel Mode - the unit is about 0.1%.
      {RegisterKey::CURRENT_VOLTAGE, {144, 2, 0, 0}},                    //  Present Voltage - This value is 10 times larger than the actual voltage
      {RegisterKey::CURRENT_TEMPERATURE, {146, 1, 0, 0}},                //  Present Temperature in degree
      {RegisterKey::MOVING, {122, 1, 0, 0}},                             //  Movement Status
      {RegisterKey::GOAL_POSITION, {116, 4, 0, 1}},                      //  Target Position
      {RegisterKey::MOVING_SPEED, {104, 4, 0, 0}},                       //  Moving Speed | Wheel Mode: 0 ~ 2,047(0x7FF) can be used, the unit is about 0.1%. | Join Mode: 0 ~ 1,023(0x3FF) can be used, and the unit is about 0.111rpm.
      {RegisterKey::CURRENT_LOAD, {126, 2, 0, 0}},                       //  Present Load - unit is 0.1%.
      {RegisterKey::TORQUE_ENABLE, {64, 1, 0, 1}},                       //  Motor Torque On/Off
      {RegisterKey::STATUS_RETURN_LEVEL, {68, 1, 0, 2}},                 //  Select Types of Status Return (0 - PING, 1 PING READ or 2 ALL)
      {RegisterKey::ACCELERATION_VALUE_OF_PROFILE, {108, 4, 0, 32767}},  //  Sets acceleration of the Profile (0 stand for infinite acceleration)
      {RegisterKey::VELOCITY_VALUE_OF_PROFILE, {112, 4, 0, 32767}},      //  Sets speed of the Profile (0 stand for infinite speed)
      {RegisterKey::TIME_MS, {120, 2, 0, 32767}}                         //  Count Time in millisecond
    };
    return map;
  }
  return std::map<RegisterKey, ServoRegister>();
}

}  // namespace dynamixel
}  // namespace isaac
