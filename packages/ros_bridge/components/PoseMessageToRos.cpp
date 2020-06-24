/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PoseMessageToRos.hpp"

#include "messages/math.hpp"

namespace isaac {
namespace ros_bridge {

bool PoseMessageToRos::protoToRos(PoseTreeEdgeProto::Reader reader, const ros::Time& ros_time,
                                  geometry_msgs::PoseWithCovarianceStamped& ros_message) {
  // Read data from Isaac type
  const Pose3d pose = FromProto(reader.getPose());
  const auto& translation = pose.translation;
  const auto& rotation = pose.rotation.quaternion();

  // Populate data for ROS type
  ros_message.header.stamp = ros_time;
  ros_message.header.frame_id = get_frame_id();
  ros_message.pose.pose.position.x = translation.x();
  ros_message.pose.pose.position.y = translation.y();
  ros_message.pose.pose.position.z = translation.z();
  ros_message.pose.pose.orientation.x = rotation.x();
  ros_message.pose.pose.orientation.y = rotation.y();
  ros_message.pose.pose.orientation.z = rotation.z();
  ros_message.pose.pose.orientation.w = rotation.w();
  if (get_report_success()) {
    reportSuccess();
  }
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
