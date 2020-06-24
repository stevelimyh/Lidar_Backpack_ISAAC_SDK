/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>

#include "engine/alice/alice_codelet.hpp"
#include "include/laikago_sdk/laikago_sdk.hpp"
#include "messages/imu.capnp.h"
#include "messages/state.capnp.h"

namespace isaac {
namespace laikago {

// A driver for Laikago, a quadruped robot designed by Unitree Roboitcs. The driver receives
// holonomic control commands and translates them to laikago sdk high level velocity command
// which it outputs. On the other side it receives laikago states (base velocity), translates
// them into holonomic base state and publishes them.
class LaikagoDriver : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // The desired motion of Laikago of type messages::HolonomicBaseControls
  ISAAC_PROTO_RX(StateProto, base_command);

  // The state of Laikago of type messages::HolonomicBaseDynamics
  ISAAC_PROTO_TX(StateProto, base_state);

  // The imu proto has linear and angular acceleration data
  ISAAC_PROTO_TX(ImuProto, imu);

  // Maximum linear speed robot is allowed to travel with
  ISAAC_PARAM(double, speed_limit_linear, 0.6);

  // Maximum angular speed robot is allowed to rotate with
  ISAAC_PARAM(double, speed_limit_angular, 0.8);

  // Minimum command speed to switch to walking mode
  ISAAC_PARAM(double, min_command_speed, 0.01);

  // Scale backward speed.
  // The laikago walks backward slower than forward given the same command value.
  // To approximately compensate the bias, we scale up the command speed
  ISAAC_PARAM(double, scale_back_speed, 2.0);

  // Scale side walk speed.
  // The laikago side walk speed is about three times slower than command.
  // To approximately compensate the tracking error, we scale up the command speed
  ISAAC_PARAM(double, scale_side_speed, 3.0);

 private:
  // Failsafe for safety
  alice::Failsafe* failsafe_;
  // Laikago direction command
  ::laikago::HighCmd cmd_;
  // Laikago odometry states
  ::laikago::HighState state_;
  // Class to define Laikago control level
  std::unique_ptr<::laikago::Control> control_;
  // UDP for robot communication
  std::unique_ptr<::laikago::UDP> udp_;
};

}  // namespace laikago
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::laikago::LaikagoDriver);
