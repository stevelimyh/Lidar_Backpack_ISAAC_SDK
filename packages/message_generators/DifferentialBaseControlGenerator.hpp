/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"

namespace isaac {
namespace message_generators {

// Generates periodic differential base states with specified parameters
class DifferentialBaseControlGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // A StateProto representing a navigation::DifferentialBaseControl state which is populated with
  // values specified via parameters
  ISAAC_PROTO_TX(StateProto, command);

  // Linear speed in outgoing state message
  ISAAC_PARAM(double, linear_speed, 0.0);
  // Angular speed in outgoing state message
  ISAAC_PARAM(double, angular_speed, 0.0);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::DifferentialBaseControlGenerator);
