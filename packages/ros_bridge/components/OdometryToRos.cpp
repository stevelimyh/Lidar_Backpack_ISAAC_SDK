/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "OdometryToRos.hpp"

#include "messages/math.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace isaac {
namespace ros_bridge {

bool OdometryToRos::protoToRos(Odometry2Proto::Reader reader, const ros::Time& ros_time,
                               nav_msgs::Odometry& ros_message) {
  // Read data from Isaac type
  const Pose2d odometry_T_robot = FromProto(reader.getOdomTRobot());
  const double linear_speed = reader.getSpeed().getX();
  const double angular_speed = reader.getAngularSpeed();

  // Populate data for ROS type
  ros_message.header.stamp = ros_time;
  // From http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html:
  // "The pose in this message should be specified in the coordinate frame given by header.frame_id.
  // The twist in this message should be specified in the coordinate frame given by the
  // child_frame_id"
  ros_message.header.frame_id = get_pose_frame();
  ros_message.child_frame_id = get_twist_frame();
  ros_message.pose.pose.position.x = odometry_T_robot.translation.x();
  ros_message.pose.pose.position.y = odometry_T_robot.translation.y();
  ros_message.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, odometry_T_robot.rotation.angle());
  ros_message.pose.pose.orientation.x = q.x();
  ros_message.pose.pose.orientation.y = q.y();
  ros_message.pose.pose.orientation.z = q.z();
  ros_message.pose.pose.orientation.w = q.w();
  ros_message.twist.twist.linear.x = linear_speed;
  ros_message.twist.twist.angular.z = angular_speed;
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
