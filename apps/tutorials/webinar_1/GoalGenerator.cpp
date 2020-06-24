/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "GoalGenerator.hpp"

#include "messages/math.hpp"

namespace isaac {
namespace tutorials {

namespace {

// Goal tolerance in meters
constexpr double kGoalTolerance = 0.1;

}  // namespace

void GoalGenerator::start() {
  goal_timestamp_ = 0;
  goal_position_ = Vector2d::Zero();
  tickPeriodically();
}

void GoalGenerator::publishGoal(const Vector2d& position) {
  // Save the timestamp to later check it against the feedback timestamp
  goal_timestamp_ = getTickTimestamp();
  // Update the last goal information to avoid transmitting repeated messages
  goal_position_ = position;
  // Show the new goal on WebSight
  show("goal_timestamp", goal_timestamp_);
  show("goal_position_x", goal_position_.x());
  show("goal_position_y", goal_position_.y());
  // Publish a goal with the new goal location
  auto goal_proto = tx_goal().initProto();
  goal_proto.setStopRobot(false);
  ToProto(Pose2d::Translation(position), goal_proto.initGoal());
  // Use a goal tolerance of kGoalTolerance meters
  goal_proto.setTolerance(kGoalTolerance);
  // The goal we publish is with respect to the global "world" coordinate frame
  goal_proto.setGoalFrame("world");
  tx_goal().publish(goal_timestamp_);
}

void GoalGenerator::tick() {
  // Read desired position parameter
  const Vector2d position = get_desired_position();
  // Publish goal, if there has been a location change
  if (isFirstTick() || (position - goal_position_).norm() > kGoalTolerance) {
    publishGoal(position);
  }

  // Process feedback
  rx_feedback().processLatestNewMessage(
      [this](auto feedback_proto, int64_t pubtime, int64_t acqtime) {
        // Check if this feedback is associated with the last goal we transmitted
        if (goal_timestamp_ != acqtime) {
          return;
        }
        const bool arrived = feedback_proto.getHasArrived();
        // Show arrival information on WebSight
        show("arrived", arrived ? 1.0 : 0.0);
      });
}

}  // namespace tutorials
}  // namespace isaac
