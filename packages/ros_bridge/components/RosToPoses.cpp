/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToPoses.hpp"

#include <memory>
#include <string>

namespace isaac {
namespace ros_bridge {

void RosToPoses::start() {
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

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  tickPeriodically();
}

void RosToPoses::tick() {
  // Check with RosNode
  ASSERT(ros_node_, "Logic error");
  if (auto maybe_error = ros_node_->checkBeforeInterface()) {
    reportFailure(maybe_error->c_str());
    return;
  }

  ASSERT(tf2_buffer_, "Logic error");
  ASSERT(tf2_listener_, "Logic error");
  for (const IsaacRosPoseMapping& pose_mapping : get_pose_mappings()) {
    try {
      // Get transformation from ROS
      const geometry_msgs::TransformStamped transformStamped = tf2_buffer_->lookupTransform(
          pose_mapping.ros_pose.lhs_frame, pose_mapping.ros_pose.rhs_frame, ros::Time(0));
      const geometry_msgs::Transform& transform = transformStamped.transform;
      const geometry_msgs::Quaternion& rotation = transform.rotation;
      const geometry_msgs::Vector3& translation = transform.translation;

      // Create Isaac pose
      const Pose3d pose{
          SO3d::FromQuaternion(Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)),
          Vector3d{translation.x, translation.y, translation.z}};
      node()->pose().set(pose_mapping.isaac_pose.lhs_frame, pose_mapping.isaac_pose.rhs_frame, pose,
                         getTickTime());
    } catch (...) {
      // tf2::TransformException will be thrown if transformation could not be read
    }
  }
}

void RosToPoses::stop() {
  // Reset listener before buffer
  tf2_listener_ = nullptr;
  tf2_buffer_ = nullptr;
}

}  // namespace ros_bridge
}  // namespace isaac
