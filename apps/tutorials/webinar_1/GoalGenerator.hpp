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
#include "engine/core/math/pose2.hpp"
#include "messages/differential_base.capnp.h"

namespace isaac {
namespace tutorials {

// Completed version of MyGoalGenerator codelet for the first ISAAC webinar.
// A goal in world frame at desired location is transmitted as goal.
// This simple codelet illustrates various fundamentals of ISAAC, including
// * start and tick functions,
// * Message transmission and receival,
// * Parameters that are configurable through json or Sight.
class GoalGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output goal for the robot
  ISAAC_PROTO_TX(Goal2Proto, goal);
  // Feedback about our progress towards the goal
  ISAAC_PROTO_RX(Goal2FeedbackProto, feedback);

  // Desired x and y position of the robot on map in meters
  ISAAC_PARAM(Vector2d, desired_position, Vector2d(9.0, 25.0));

 private:
  // Publishes a goal message with given target position.
  void publishGoal(const Vector2d& position);

  // Location of the last goal that is transmitted
  Vector2d goal_position_;
  // Timestamp of the last goal that is transmitted
  int64_t goal_timestamp_;
};

}  // namespace tutorials
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::tutorials::GoalGenerator);
