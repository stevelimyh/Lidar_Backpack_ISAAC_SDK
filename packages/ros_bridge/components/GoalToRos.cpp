/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "GoalToRos.hpp"

#include <string>

#include "messages/math.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace isaac {
namespace ros_bridge {

bool GoalToRos::protoToRos(Goal2Proto::Reader reader, const ros::Time& ros_time,
                           move_base_msgs::MoveBaseActionGoal& ros_message) {
  // Read data from Isaac type
  Pose2d pose;
  std::string frame;
  if (reader.getStopRobot()) {
    // Stop the robot by sending current pose as goal
    pose = Pose2d::Identity();
    frame = get_robot_frame();
  } else {
    pose = FromProto(reader.getGoal());
    frame = get_goal_frame();
  }

  // Publish goal data to ROS.
  ros_message.header.stamp = ros_time;
  ros_message.goal.target_pose.header.frame_id = frame;
  ros_message.goal.target_pose.header.stamp = ros_time;
  ros_message.goal.target_pose.pose.position.x = pose.translation.x();
  ros_message.goal.target_pose.pose.position.y = pose.translation.y();
  ros_message.goal.target_pose.pose.position.z = 0.0;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, pose.rotation.angle());
  ros_message.goal.target_pose.pose.orientation.x = quaternion.x();
  ros_message.goal.target_pose.pose.orientation.y = quaternion.y();
  ros_message.goal.target_pose.pose.orientation.z = quaternion.z();
  ros_message.goal.target_pose.pose.orientation.w = quaternion.w();
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
