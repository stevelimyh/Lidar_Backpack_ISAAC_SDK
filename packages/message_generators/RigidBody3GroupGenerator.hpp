/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/rigid_body_3_group.capnp.h"

namespace isaac {
namespace message_generators {

// Publishes messages with a single body using configured values
class RigidBody3GroupGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output group with a single body
  ISAAC_PROTO_TX(RigidBody3GroupProto, bodies);

  // Name of the body
  ISAAC_PARAM(std::string, body_name, "dummy_body");
  // Reference frame for the body
  ISAAC_PARAM(std::string, reference_frame, "world");
  // Pose of the body with respect to the reference frame
  ISAAC_PARAM(Pose3d, pose, Pose3d::Identity());
  // Linear velocity of the body
  ISAAC_PARAM(Vector3d, linear_velocity, Vector3d::Zero());
  // Angular velocity of the body
  ISAAC_PARAM(Vector3d, angular_velocity, Vector3d::Zero());
  // Linear acceleration of the body
  ISAAC_PARAM(Vector3d, linear_acceleration, Vector3d::Zero());
  // Angular acceleration of the body
  ISAAC_PARAM(Vector3d, angular_acceleration, Vector3d::Zero());
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::RigidBody3GroupGenerator)
