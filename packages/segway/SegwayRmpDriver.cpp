/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SegwayRmpDriver.hpp"

#include <cmath>

#include "engine/alice/components/Failsafe.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"
#include "packages/segway/gems/segway.hpp"

namespace isaac {

namespace {

constexpr double kMaxSafeSpeedLimit = 2.0;
constexpr double kMaxSafeTurningRateLimit = 2.0;

}  // namespace

SegwayRmpDriver::SegwayRmpDriver() {}

SegwayRmpDriver::~SegwayRmpDriver() {}

void SegwayRmpDriver::start() {
  failsafe_ = node()->getComponent<alice::Failsafe>();
  tickBlocking();
  segway_.reset(new drivers::Segway(get_ip(), get_port()));
  segway_->start();
  // beep
  segway_->sendCommandU(0x0501, 31, 1);
  segway_->sendCommandU(0x0501, 32, 5);
}

void SegwayRmpDriver::tick() {
  if (rx_segway_cmd().available()) {
    // Get the commanded values and make sure they are safe
    messages::DifferentialBaseControl command;
    ASSERT(FromProto(rx_segway_cmd().getProto(), rx_segway_cmd().buffers(), command),
           "Failed to parse rx_segway_cmd");
    double safe_speed = command.linear_speed();
    const double speed_limit_linear = get_speed_limit_linear();
    const double speed_limit_angular = get_speed_limit_angular();
    if (speed_limit_linear <= 0.0 || speed_limit_linear >= kMaxSafeSpeedLimit) {
      LOG_ERROR("Linear speed limit is %f. It needs to be a positive number smaller than %f.",
                speed_limit_linear, kMaxSafeSpeedLimit);
      return;
    }
    if (speed_limit_angular <= 0.0 || speed_limit_angular >= kMaxSafeTurningRateLimit) {
      LOG_ERROR("Angular speed limit is %f. It needs to be a positive number smaller than %f.",
                speed_limit_angular, kMaxSafeTurningRateLimit);
      return;
    }
    if (std::abs(safe_speed) > speed_limit_linear) {
      LOG_WARNING("Speed command out of safe bounds. Clamped from %f to %f.", safe_speed,
                  speed_limit_linear);
      safe_speed = std::copysign(speed_limit_linear, command.linear_speed());
    }
    double safe_turning_rate = command.angular_speed();
    if (std::abs(safe_turning_rate) > speed_limit_angular) {
      LOG_WARNING("Turning rate command out of safe bounds. Clamped from %f to %f.",
                  safe_turning_rate, speed_limit_angular);
      safe_turning_rate = std::copysign(speed_limit_angular, command.angular_speed());
    }

    // Positive rate means clockwise turn for the segway
    safe_turning_rate *= -1.0;
    // apply a flip if requested
    if (get_flip_orientation()) {
      safe_speed *= -1.0;
    }
    // stop robot if the failsafe is triggered
    if (!failsafe_->isAlive()) {
      safe_speed = 0.0;
      safe_turning_rate = 0.0;
    }
    // Send commands to the segway
    const bool res = segway_->sendSpeedCommand(safe_speed, safe_turning_rate);
    if (!res) {
      LOG_ERROR("Failed to send command");
    }
  }

  auto state = segway_->getSegwayState();
  // Positive rate means clockwise turn for the segway
  state.differential_wheel_vel_rps *= -1.0;
  if (get_flip_orientation()) {
    state.linear_accel_msp2 *= -1.0;
    state.linear_vel_mps *= -1.0;
  }

  // publish current state of segway
  messages::DifferentialBaseDynamics segway_state;
  segway_state.linear_speed() = state.linear_vel_mps;
  segway_state.angular_speed() = state.differential_wheel_vel_rps;
  segway_state.linear_acceleration() = state.linear_accel_msp2;
  segway_state.angular_acceleration() = 0.0;
  ToProto(segway_state, tx_segway_state().initProto(), tx_segway_state().buffers());

  tx_segway_state().publish();

  // visualization
  show("linear_accel_msp2", state.linear_accel_msp2);
  show("linear_vel_mps", state.linear_vel_mps);
  show("differential_wheel_vel_rps", state.differential_wheel_vel_rps);
  show("min_propulsion_batt_soc", state.min_propulsion_batt_soc);
  show("aux_batt_soc", state.aux_batt_soc);
}

void SegwayRmpDriver::stop() {
  segway_->sendCommandU(0x0501, 31, 2);
  segway_->sendCommandU(0x0501, 32, 4);
  segway_->stop();
}

}  // namespace isaac
