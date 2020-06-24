/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <map>

#include "engine/core/byte.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {
namespace dynamixel {
namespace {
constexpr double kRpmRatio = 60.0;
}  // namespace

// Serial port baudrate
// The integer values are mapped to the external dynamixel library
enum class Baudrate  {
  k4_5M = 7,
  k4M = 6,
  k3M = 5,
  k2M = 4,
  k1M = 3,
  k115200 = 2,
  k57600 = 1,
  k9600 = 0,
  kInvalid = -1
};

// Dynamixel model
// https://www.trossenrobotics.com/robot-servos
enum class Model {
  AX12A = 0,
  XM430 = 1,
  MX12W = 2,
  XC430 = 3,
  INVALID = -1
};

// Servo register
struct ServoRegister {
  uint16_t address;
  uint8_t size;
  int min;
  int max;
};

// Register keys
enum RegisterKey {
  MODEL_NUMBER,                   // 0
  FIRMWARE_VERSION,               // 1
  DYNAMIXEL_ID,                   // 2
  BAUDRATE,                       // 3
  RETURN_DELAY_TIME,              // 4
  DRIVE_MODE,                     // 5
  OPERATING_MODE,                 // 6
  PROTOCOL_VERSION,               // 7
  CW_ANGLE_LIMIT,                 // 8
  CCW_ANGLE_LIMIT,                // 9
  HOME_POSITION_OFFSET,           // 10
  TEMPERATURE_LIMIT,              // 11
  MIN_VOLTAGE_LIMIT,              // 12
  MAX_VOLTAGE_LIMIT,              // 13
  MAX_TORQUE,                     // 14
  TORQUE_LIMIT,                   // 15
  STATUS_RETURN_LEVEL,            // 16
  ALARM_LED,                      // 17
  SHUTDOWN,                       // 18
  TORQUE_ENABLE,                  // 19
  GOAL_POSITION,                  // 20
  MOVING_SPEED,                   // 21
  CURRENT_POSITION,               // 22
  CURRENT_SPEED,                  // 23
  CURRENT_LOAD,                   // 24
  CURRENT_TEMPERATURE,            // 25
  CURRENT_VOLTAGE,                // 26
  ACCELERATION_VALUE_OF_PROFILE,  // 27
  VELOCITY_VALUE_OF_PROFILE,      // 27
  TIME_MS,                        // 28
  LED,                            // 29
  MOVING                          // 30
};

// Number of Dynamixel "position ticks" for a full rotation (from Dynamixel spec sheet)
constexpr int kAngleResolutionXM430 = 4096;
constexpr int kAngleResolutionXC430 = 4096;
constexpr int kAngleResolutionAX12A = 4096;
constexpr int kAngleResolutionMX12W = 4096;

// Rotations per minute per Dynamixel "speed tick" (from Dynamixel spec sheet)
constexpr double kRpmResolutionXM430 = 0.229;
constexpr double kRpmResolutionXC430 = 0.229;
constexpr double kRpmResolutionAX12A = 0.916;
constexpr double kRpmResolutionMX12W = 0.916;

// Hardware status codes (from Dynamixel spec sheet)
constexpr byte kStatusOk                = 0x00;
constexpr byte kStatusInputVoltageError = 0x01;
constexpr byte kStatusAngleLimitError   = 0x02;
constexpr byte kStatusOverheating       = 0x04;
constexpr byte kStatusRangeError        = 0x08;
constexpr byte kStatusChecksumError     = 0x10;
constexpr byte kStatusOverloaded        = 0x20;
constexpr byte kStatusInstructionError  = 0x40;

// Max valid moving speed (from Dynamixel spec sheet)
constexpr int kMaxMovingSpeed = 0x3FF;

// Flag to control moving direction (from Dynamixel spec sheet)
constexpr int kMovingSpeedDirectionFlagAX12A = 0x400;
constexpr int kMovingSpeedDirectionFlagMX12W = 0x400;

// Inits registers depending on the Servo Model
// For instance for AX12
// http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table
std::map<RegisterKey, ServoRegister> InitRegisters(Model model);

// Map enum values to integer values for the external dynamixel library
inline int GetBaudrateValue(Baudrate baudrate) {
  switch (baudrate) {
    case Baudrate::k4_5M:
      return 4'500'000;
    case Baudrate::k4M:
      return 4'000'000;
    case Baudrate::k3M:
      return 3'000'000;
    case Baudrate::k2M:
      return 2'000'000;
    case Baudrate::k1M:
      return 1'000'000;
    case Baudrate::k115200:
      return 115'200;
    case Baudrate::k57600:
      return 57'600;
    case Baudrate::k9600:
      return 9'600;
    case Baudrate::kInvalid:
      break;
  }
  return 0;
}

// Gets protocol for instructions and status packet
inline float GetProtocol(Model model) {
  switch (model) {
    case Model::AX12A:
      return 1.0;
    case Model::MX12W:
      return 1.0;
    case Model::XM430:
      return 2.0;
    case Model::XC430:
      return 2.0;
    case Model::INVALID:
      break;
  }
  return 0.0;
}

// Gets Angle value depending on motor model
inline int GetAngleResolution(Model model) {
  switch (model) {
    case Model::AX12A:
      return kAngleResolutionAX12A;
    case Model::MX12W:
      return kAngleResolutionMX12W;
    case Model::XM430:
      return kAngleResolutionXM430;
    case Model::XC430:
      return kRpmResolutionXC430;
    case Model::INVALID:
      break;
  }
  return 0;
}

// Gets RPM value depending on motor model
inline double GetRpmResolution(Model model) {
  switch (model) {
    case Model::AX12A:
      return kRpmResolutionAX12A;
    case Model::MX12W:
      return kRpmResolutionMX12W;
    case Model::XM430:
      return kRpmResolutionXM430;
    case Model::XC430:
      return kRpmResolutionXM430;
    case Model::INVALID:
      break;
  }
  return 0.0;
}
// Converts a dynamixel joint position to the corresponding angle in radians.
inline double TicksToAngle(Model model, int position_ticks) {
  const int angle_resolution = GetAngleResolution(model);
  const double factor = TwoPi<double> / static_cast<double>(angle_resolution);
  return factor * static_cast<double>(position_ticks);
}

// Inverse of TicksToAngle
inline int AngleToTicks(Model model, double angle) {
  const double factor = static_cast<double>(GetAngleResolution(model)) / TwoPi<double>;
  return static_cast<int>(factor * WrapTwoPi(angle));
}

// Converts a dynamixel joint speed to the corresponding angular speed in radians per second.
inline double TicksToAngularSpeed(Model model, int speed_ticks) {
  const double factor = (GetRpmResolution(model) * TwoPi<double>) / kRpmRatio;
  switch (model) {
    case Model::AX12A:
      if (speed_ticks & kMovingSpeedDirectionFlagAX12A) {
        speed_ticks = kMovingSpeedDirectionFlagAX12A - speed_ticks;
      }
      break;
    case Model::MX12W:
      if (speed_ticks & kMovingSpeedDirectionFlagMX12W) {
        speed_ticks = kMovingSpeedDirectionFlagMX12W - speed_ticks;
      }
      break;
    default:
      break;
  }
  return factor * static_cast<double>(speed_ticks);
}

// Depending on the dynamixel model the negative value is stored differently. This option computes
// the correct negative value associated to `value` depending on the dynamixel model.
inline void ReverseValue(Model model, int& value) {
  switch (model) {
    case Model::AX12A:
      value |= kMovingSpeedDirectionFlagAX12A;
      return;
    case Model::MX12W:
      value |= kMovingSpeedDirectionFlagMX12W;
      return;
    default:
      value = -value;
  }
}

// Inverse of TicksToSpeed
inline int AngularSpeedToTicks(Model model, double angular_speed) {
  const double factor = kRpmRatio / (GetRpmResolution(model) * TwoPi<double>);
  int ticks = std::min(kMaxMovingSpeed, static_cast<int>(factor * std::abs(angular_speed)));
  if (angular_speed < 0) { ReverseValue(model, ticks); }
  return ticks;
}

}  // namespace dynamixel
}  // namespace isaac
