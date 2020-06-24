/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PosesToRos.hpp"

#include <memory>
#include <string>

namespace isaac {
namespace ros_bridge {

void PosesToRos::start() {
  // Get RosNode pointer
  const std::string ros_node_name = get_ros_node();
  ros_node_ = node()->app()->getNodeComponentOrNull<RosNode>(ros_node_name);
  if (!ros_node_) {
    reportFailure("No RosNode component named '%s'", ros_node_name.c_str());
    return;
  }
  // Check with RosNode
  if (auto maybe_error = ros_node_->checkBeforeInterface()) {
    reportFailure(maybe_error->c_str());
    return;
  }

  // tf2_broadcaster_ needs to be constructed after ros_node_ reports success.
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

  tickPeriodically();
}

void PosesToRos::tick() {
  // Check with RosNode
  ASSERT(ros_node_, "Logic error");
  if (auto maybe_error = ros_node_->checkBeforeInterface()) {
    reportFailure(maybe_error->c_str());
    return;
  }

  ASSERT(tf2_broadcaster_, "Logic error");
  for (const IsaacRosPoseMapping& pose_mapping : get_pose_mappings()) {
    // Read pose from Isaac Pose Tree
    const auto maybe_pose = node()->pose().tryGet(pose_mapping.isaac_pose.lhs_frame,
                                                  pose_mapping.isaac_pose.rhs_frame, getTickTime());
    if (!maybe_pose) {
      // Pose may not be available in the pose tree yet. Continue with other poses.
      continue;
    }
    const auto& translation = maybe_pose->translation;
    const auto& rotation = maybe_pose->rotation.quaternion();

    // Publish transformation to ROS
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = pose_mapping.ros_pose.lhs_frame;
    transformStamped.child_frame_id = pose_mapping.ros_pose.rhs_frame;
    transformStamped.transform.translation.x = translation.x();
    transformStamped.transform.translation.y = translation.y();
    transformStamped.transform.translation.z = translation.z();
    transformStamped.transform.rotation.x = rotation.x();
    transformStamped.transform.rotation.y = rotation.y();
    transformStamped.transform.rotation.z = rotation.z();
    transformStamped.transform.rotation.w = rotation.w();
    tf2_broadcaster_->sendTransform(transformStamped);
  }
}

void PosesToRos::stop() {
  tf2_broadcaster_ = nullptr;
}

}  // namespace ros_bridge
}  // namespace isaac
