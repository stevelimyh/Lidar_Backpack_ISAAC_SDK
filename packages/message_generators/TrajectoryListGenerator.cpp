/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "TrajectoryListGenerator.hpp"

#include <vector>

#include "messages/math.hpp"

namespace isaac {
namespace message_generators {

void TrajectoryListGenerator::start() {
  tickPeriodically();
}

void TrajectoryListGenerator::tick() {
  // Prepare a message with a trajectory to publish.
  auto trajectories_proto = tx_trajectories().initProto();
  auto trajectories = trajectories_proto.initTrajectories(1);

  // A trajectory as a vertical helix spinning over time.
  const double angle_start = getTickCount();
  trajectories[0].setFrame(get_frame());

  auto states = trajectories[0].initStates(get_position_count());
  for (int state_index = 0; state_index < get_position_count(); state_index++) {
    const double angle =
        get_position_delta_angle() * (angle_start - static_cast<double>(state_index));
    ToProto(Vector3d{std::cos(angle) * get_helix_radius(), std::sin(angle) * get_helix_radius(),
                     static_cast<double>(state_index) / static_cast<double>(get_position_count())},
            states[state_index]);
  }

  tx_trajectories().publish();
}

}  // namespace message_generators
}  // namespace isaac
