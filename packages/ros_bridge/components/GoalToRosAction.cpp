/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "GoalToRosAction.hpp"

#include <memory>

#include "messages/math.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace isaac {
namespace ros_bridge {

namespace {
// waitForServer() will timeout after this many seconds, if it doesn't return true before that.
constexpr double kWaitForServerTimeout = 0.1;
}  // namespace

void GoalToRosAction::start() {
  if (!ros::isInitialized()) {
    reportFailure("ros::init should be called by RosNode. Please check your behavior tree.");
    return;
  }

  action_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
      get_action_name());

  // Tick periodically, not on message since we check whether the robot has arrived.
  tickPeriodically();
}

void GoalToRosAction::tick() {
  if (!action_client_->waitForServer(ros::Duration(kWaitForServerTimeout))) {
    return;
  }

  convertAndSendGoal();
  publishFeedback();
}

void GoalToRosAction::stop() {
  action_client_ = nullptr;
}

void GoalToRosAction::publishFeedback() {
  const auto maybe_robot_T_frame =
      node()->pose().tryGet(get_robot_frame_isaac(), last_goal_->frame_name, getTickTime());
  if (!maybe_robot_T_frame) {
    return;
  }

  auto proto = tx_feedback().initProto();
  proto.setHasGoal(true);
  ToProto(maybe_robot_T_frame->toPose2XY() * last_goal_->frame_T_goal, proto.initRobotTGoal());
  proto.setHasArrived(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  proto.setIsStationary(isStationary());
  // Reply with the acqtime of the received goal message
  tx_feedback().publish(last_goal_->acqtime);
}

void GoalToRosAction::convertAndSendGoal() {
  rx_goal().processAllNewMessages([this](auto proto, int64_t pubtime, int64_t acqtime) {
    // Read data from Isaac type
    Goal goal;
    goal.acqtime = acqtime;
    if (proto.getStopRobot()) {
      // Stop the robot by sending current pose as goal
      goal.frame_T_goal = Pose2d::Identity();
      goal.frame_name = get_robot_frame_ros();
    } else {
      goal.frame_T_goal = FromProto(proto.getGoal());
      goal.frame_name = get_goal_frame_ros();
    }
    last_goal_ = goal;

    // Send goal data to ROS.
    move_base_msgs::MoveBaseGoal ros_goal;
    ros_goal.target_pose.header.frame_id = goal.frame_name;
    ros_goal.target_pose.header.stamp = ros::Time::now();
    ros_goal.target_pose.pose.position.x = goal.frame_T_goal.translation.x();
    ros_goal.target_pose.pose.position.y = goal.frame_T_goal.translation.y();
    ros_goal.target_pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, goal.frame_T_goal.rotation.angle());
    ros_goal.target_pose.pose.orientation.x = quaternion.x();
    ros_goal.target_pose.pose.orientation.y = quaternion.y();
    ros_goal.target_pose.pose.orientation.z = quaternion.z();
    ros_goal.target_pose.pose.orientation.w = quaternion.w();
    action_client_->sendGoal(ros_goal);
  });
}

bool GoalToRosAction::isStationary() {
  bool is_stationary = false;

  if (rx_odometry().available()) {
    auto odom_reader = rx_odometry().getProto();
    const double linear_speed_x = odom_reader.getSpeed().getX();
    const double linear_speed_y = odom_reader.getSpeed().getY();
    const double linear_speed =
        std::sqrt(std::pow(linear_speed_x, 2) + std::pow(linear_speed_y, 2));
    const double angular_speed = odom_reader.getAngularSpeed();

    const Vector2d thresholds = get_stationary_speed_thresholds();
    if (thresholds[0] < 0.0) {
      reportFailure("Negative linear stationary threshold");
    } else if (thresholds[1] < 0.0) {
      reportFailure("Negative angular stationary threshold");
    } else {
      is_stationary =
          std::abs(linear_speed) < thresholds[0] && std::abs(angular_speed) < thresholds[1];
    }
  }
  return is_stationary;
}

}  // namespace ros_bridge
}  // namespace isaac
