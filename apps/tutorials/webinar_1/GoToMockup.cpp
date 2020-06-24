/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "GoToMockup.hpp"

#include <string>

#include "messages/math.hpp"

namespace isaac {
namespace tutorials {

namespace {

// Stopwatch to change arrival status in the feedback
constexpr char kSleepHelper[] = "arrival-helper";

}  // namespace

void GoToMockup::start() {
  maybe_goal_ = std::nullopt;
  tickPeriodically();
}

void GoToMockup::tick() {
  publishFeedback();
  processGoal();
}

void GoToMockup::publishFeedback() {
  // Do not publish a feedback if no goal message is received yet.
  if (!maybe_goal_) {
    return;
  }

  // Check if we "arrived"
  bool arrived = false;
  if (stopwatch(kSleepHelper).read() > get_time_until_arrival()) {
    arrived = true;
    stopwatch(kSleepHelper).stop();
  }

  // Send a dummy feedback message
  auto proto_out = tx_feedback().initProto();
  proto_out.setHasGoal(true);
  ToProto(Pose2d::Identity(), proto_out.initRobotTGoal());
  proto_out.setHasArrived(arrived);
  proto_out.setIsStationary(false);
  tx_feedback().publish(maybe_goal_->acqtime);
}

void GoToMockup::processGoal() {
  // Process all messages received
  rx_goal().processAllNewMessages([this](auto proto, int64_t pubtime, int64_t acqtime) {
    // Process and print goal message that is received
    if (proto.getStopRobot()) {
      LOG_WARNING("Stopping the robot");
      return;
    }
    const Pose2d goal_pose = FromProto(proto.getGoal());
    const double goal_tolerance = proto.getTolerance();
    const std::string goal_frame_name = proto.getGoalFrame();
    const Goal new_goal = Goal{acqtime, goal_frame_name, goal_pose};
    // If the goal location has changed, print the new goal to console and restart the timer to
    // imitate "arrival". Note that we are ignoring the angle information in this mock-up.
    if (!maybe_goal_ || new_goal.frame_name != maybe_goal_->frame_name ||
        (new_goal.frame_T_goal.translation - maybe_goal_->frame_T_goal.translation).norm() >
            goal_tolerance) {
      LOG_INFO("Heading to a goal defined in '%s' frame: p=(%f, %f), q=%f", goal_frame_name.c_str(),
               goal_pose.translation.x(), goal_pose.translation.y(), goal_pose.rotation.angle());
      // Restart the timer. We arrive get_time_until_arrival() seconds after receiving a new goal
      // location.
      stopwatch(kSleepHelper).stop();
      stopwatch(kSleepHelper).start();
    }
    // Save the goal information
    maybe_goal_ = Goal{acqtime, goal_frame_name, goal_pose};
  });
}

}  // namespace tutorials
}  // namespace isaac
