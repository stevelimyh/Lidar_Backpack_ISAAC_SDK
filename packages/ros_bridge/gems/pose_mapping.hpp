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

#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace ros_bridge {

// A pose name, denoted by lhs_t_rhs, is defined by two frames.
struct PoseName {
  // Name of the reference frame of the left side of the pose
  std::string lhs_frame;
  // Name of the reference frame of the right side of the pose
  std::string rhs_frame;
};

// Poses in Isaac and ROS that correspond to each other, e.g.,
// odom_T_robot of Isaac may correspond to odom_T_base_footprint
// of ROS.
struct IsaacRosPoseMapping {
  PoseName isaac_pose;
  PoseName ros_pose;
};

inline void from_json(const nlohmann::json& j, PoseName& pose_name) {
  j.at("lhs_frame").get_to(pose_name.lhs_frame);
  j.at("rhs_frame").get_to(pose_name.rhs_frame);
}

inline void to_json(nlohmann::json& j, const PoseName& pose_name) {
  j = nlohmann::json{{"lhs_frame", pose_name.lhs_frame}, {"rhs_frame", pose_name.rhs_frame}};
}

inline void from_json(const nlohmann::json& j, IsaacRosPoseMapping& pose_mapping) {
  j.at("isaac_pose").get_to(pose_mapping.isaac_pose);
  j.at("ros_pose").get_to(pose_mapping.ros_pose);
}

inline void to_json(nlohmann::json& j, const IsaacRosPoseMapping& pose_mapping) {
  j = nlohmann::json{{"isaac_pose", pose_mapping.isaac_pose}, {"ros_pose", pose_mapping.ros_pose}};
}

}  // namespace ros_bridge
}  // namespace isaac
