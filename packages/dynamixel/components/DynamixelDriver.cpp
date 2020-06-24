/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "DynamixelDriver.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/alice/components/Failsafe.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"
#include "packages/dynamixel/gems/dynamixel.hpp"

namespace isaac {
namespace dynamixel {

namespace {

// Max torque limit of dynamixels
constexpr int kMaxTorqueLimit = 0x3FF;

}  // namespace

void DynamixelDriver::start() {
  // Initialize a failsafe
  failsafe_ = node()->getComponentOrNull<alice::Failsafe>();
  if (failsafe_ == nullptr) {
    LOG_WARNING("Running the Dynamixel driver without a failsafe is not recommended");
  }

  // Initialize Dynamixel motors
  dynamixel_.reset(
      new Dynamixel(get_port().c_str(), get_baudrate(), get_servo_model()));

  if (!initializeServoIds()) return;

  if (!enableDynamixels(get_control_mode())) return;
  tickPeriodically();
}

void DynamixelDriver::tick() {
  Tensor1d command(servo_ids_.size());

  // Special helper debug mode which sends constant speed to motors
  if (get_debug_mode()) {
    Fill(command, get_debug_speed());
  } else {
    if (!receiveCommand(command.view())) {
      Fill(command, 0.0);
    }
  }

  if (!writeCommand(command.const_view())) {
    return;
  }

  readAndPublishState();
}

void DynamixelDriver::stop() {
  disableDynamixels();
}

bool DynamixelDriver::initializeServoIds() {
  const std::vector<int> config_servo_ids = get_servo_ids();
  servo_ids_.clear();
  servo_ids_.reserve(config_servo_ids.size());
  for (size_t i = 0; i < config_servo_ids.size(); i++) {
    const int servo_id = config_servo_ids[i];
    // Check that IDs are in the range [0, 255]. 254 is broadcast to all
    if (servo_id < 0 || 255 < servo_id) {
      reportFailure("Invalid Dynamixel servo ID: %d", servo_id);
      return false;
    }
    // Check that IDs are unique
    for (size_t j = i + 1; j < config_servo_ids.size(); j++) {
      if (servo_id == config_servo_ids[j]) {
        reportFailure("Invalid duplicated Dynamixel servo ID: %d", servo_id);
        return false;
      }
    }
    servo_ids_.push_back(servo_id);
  }

  return true;
}

bool DynamixelDriver::enableDynamixels(DynamixelMode mode) {
  double torqueLimit = get_torque_limit();
  if (torqueLimit < 0.0 || 1.0 < torqueLimit) {
    reportFailure("Torque limit of %f is out of range. Must be in [0,1]");\
    return false;
  }

  for (const uint8_t servo : servo_ids_) {
    dynamixel_->setControlMode(servo, mode);

    dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
    dynamixel_->writeRegister(servo, RegisterKey::CW_ANGLE_LIMIT, 0);
    dynamixel_->writeRegister(servo, RegisterKey::CCW_ANGLE_LIMIT, 0);

    dynamixel_->writeRegister(servo, RegisterKey::TORQUE_LIMIT,
                              static_cast<int>(torqueLimit * kMaxTorqueLimit));

    // setting a moving speed can enable torque, so it should be the last step before
    // we manually enable torque
    dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
    dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 1);
  }

  return true;
}

bool DynamixelDriver::receiveCommand(TensorView1d command) {
  if (failsafe_ && !failsafe_->isAlive()) {
    // Failsafe is triggered. Stop the robot.
    return false;
  }

  if (!rx_command().available()) {
    // No command is available. Stop the robot.
    return false;
  }

  if (getTickTime() - ToSeconds(rx_command().acqtime()) > get_command_timeout()) {
    // Command timed out. Stop the robot.
    return false;
  }

  // Parse the state message and verify tensor type
  CpuUniversalTensorConstView pack;
  if (!FromProto(rx_command().getProto().getPack(), rx_command().buffers(), pack)) {
    reportFailure("Failed to parse command message.");
    return false;
  }
  const auto maybe = pack.tryGet<TensorConstView3d>();
  if (!maybe) {
    reportFailure("Received state does not contain a rank 3 tensor of doubles");
    return false;
  }
  if (maybe->dimensions()[2] != static_cast<int>(servo_ids_.size())) {
    reportFailure("Received a command message for %zd motors, but configured %zd motors.",
                  maybe->dimensions()[2], servo_ids_.size());
    return false;
  }

  // Retreive state
  Copy(maybe->const_slice(0).const_slice(0), command);
  return true;
}

bool DynamixelDriver::writeCommand(TensorConstView1d command) {
  // Send command to motors
  double max_speed = get_max_speed();
  if (max_speed < 0.0) {
    reportFailure("Invalid maximum speed (%f) - sending zero speeds.", max_speed);
    max_speed = 0.0;
  }

  bool success = true;
  for (size_t i = 0; i < servo_ids_.size(); i++) {
    const int servo_id = servo_ids_[i];

    // Get and clamp speed
    double servo_speed = command(i);
    if (std::abs(servo_speed) > max_speed) {
      servo_speed = std::copysign(max_speed, servo_speed);
    }

    // Write to motor
    const bool failed = dynamixel_->writeRegister(servo_id, RegisterKey::MOVING_SPEED,
                                                  dynamixel_->getAngularSpeedToTicks(servo_speed));
    success &= failed;

    // Show with sight
    const std::string key = "motor_" + std::to_string(i + 1);
    show(key + ".command", command(i));
    show(key + ".fail", failed ? 1 : 0);
  }
  // if a write failed, check the error and disable the dynamixels if necessary
  if (!success) {
    const byte status = dynamixel_->getStatus();
    const bool serious_error =
        (status & kStatusOverheating) || (status & kStatusOverloaded);

    if (serious_error) {
      reportFailure("Dynamixel serious error detected; disabling servos");
      return false;
    }
    // do not log other errors, since the dynamixel API has already logged the error
  }

  return true;
}

void DynamixelDriver::readAndPublishState() {
  Tensor3d actual_speed(1, 1, servo_ids_.size());
  for (size_t i = 0; i < servo_ids_.size(); i++) {
    const int servo_id = servo_ids_[i];
    const int current_speed_back =
        dynamixel_->readRegister(servo_id, RegisterKey::CURRENT_SPEED);
    actual_speed(0, 0, i) = dynamixel_->getTicksToAngularSpeed(current_speed_back);

    // Visualize with sight
    const std::string key = "motor_" + std::to_string(i + 1);
    show(key + ".state", actual_speed(0, 0, i));
  }
  ToProto(std::move(actual_speed), tx_state().initProto().initPack(), tx_state().buffers());
  tx_state().publish();
}

void DynamixelDriver::disableDynamixels() {
  for (const uint8_t servo : servo_ids_) {
    // disabling torque needs to be after setting the moving speed, otherwise the servos will
    // automatically enable torque
    dynamixel_->writeRegister(servo, RegisterKey::MOVING_SPEED, 0);
    dynamixel_->writeRegister(servo, RegisterKey::TORQUE_ENABLE, 0);
  }
}

}  // namespace dynamixel
}  // namespace isaac
