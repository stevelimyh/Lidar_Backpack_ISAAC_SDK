/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/ros_bridge/components/RosNode.hpp"
#include "packages/ros_bridge/gems/pose_mapping.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace isaac {
namespace ros_bridge {

// For a list of pose mappings, reads pose from Isaac Pose Tree and writes it to ROS tf2.
// Note that frame definitions between Isaac and ROS may not match, e.g., "map" frame
// of Isaac and "map" frame of ROS are typically different.
class PosesToRos : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Name of the Isaac node with RosNode component
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, ros_node, "ros_node");
  // A json object from configuration containing the poses to read from Isaac Pose Tree and write to
  // ROS.
  // Left hand side (lhs_frame) corresponds to target_frame in tf2 notation.
  // Right hand side (rhs_frame) corresponds to source_frame in tf2 notation.
  // Layout:
  //    [
  //      {
  //        {
  //          "isaac_pose": {
  //            "lhs_frame": "odom",
  //            "rhs_frame": "robot"
  //        },
  //          "ros_pose": {
  //            "lhs_frame": "odom",
  //            "rhs_frame": "base_footprint"
  //          }
  //        }
  //      }
  //    ]
  ISAAC_PARAM(std::vector<IsaacRosPoseMapping>, pose_mappings, {});

 private:
  RosNode* ros_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::PosesToRos);
