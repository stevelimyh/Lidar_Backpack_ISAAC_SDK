/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"

namespace isaac { namespace drivers { class Segway; }};

namespace isaac {

// A driver for the Segway RMP base.
class SegwayRmpDriver : public isaac::alice::Codelet {
 public:
  SegwayRmpDriver();
  ~SegwayRmpDriver();
  void start() override;
  void tick() override;
  void stop() override;

  // Linear and angular speed command for driving segway (navigation::DifferentialBaseControl type)
  ISAAC_PROTO_RX(StateProto, segway_cmd);

  // State of the segway consisting of linear and angular speeds and accelerations
  // (DifferentialBaseDynamics)
  ISAAC_PROTO_TX(StateProto, segway_state);

  // Isaac will use this IP to talk to segway
  ISAAC_PARAM(std::string, ip, "192.168.0.40");

  // Isaac will use this port to talk to segway
  ISAAC_PARAM(int, port, 8080);

  // If true, segway's forward direction will be flipped
  ISAAC_PARAM(bool, flip_orientation, true);

  // Maximum linear speed segway is allowed to travel with
  ISAAC_PARAM(double, speed_limit_linear, 1.1);

  // Maximum angular speed segway is allowed to rotate with
  ISAAC_PARAM(double, speed_limit_angular, 1.0);

 private:
  std::unique_ptr<drivers::Segway> segway_;

  alice::Failsafe* failsafe_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::SegwayRmpDriver);
