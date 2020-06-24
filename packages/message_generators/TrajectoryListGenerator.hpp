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

#include "engine/alice/components/Codelet.hpp"
#include "messages/trajectory.capnp.h"

namespace isaac {
namespace message_generators {

// TrajectoryListGenerator publishes a trajectory with made up data.
// The fake trajectory is a vertical helix, in 3D, centered on the reference frame
// origin, spinning around the Z axis.
class TrajectoryListGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // The output channel to send all generated trajectories.
  ISAAC_PROTO_TX(Vector3TrajectoryListProto, trajectories);

  // Reference frame for the generated trajectories.
  ISAAC_PARAM(std::string, frame, "world");

  // Number of positions in the generated trajectory.
  ISAAC_PARAM(int, position_count, 60);

  // The radius of the vertical helix created as the made up trajectory.
  ISAAC_PARAM(double, helix_radius, 5.0);

  // The delta angle between each positions in the generated trajectory.
  ISAAC_PARAM(double, position_delta_angle, 0.1);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::TrajectoryListGenerator);
