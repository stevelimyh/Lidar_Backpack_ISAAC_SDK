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

#include "messages/differential_base.capnp.h"
#include "nav_msgs/Odometry.h"
#include "packages/ros_bridge/components/ProtoToRosConverter.hpp"

namespace isaac {
namespace ros_bridge {

// This codelet receives odometry data within Isaac application and publishes it to ROS.
// This codelet assumes ROS subscriber is respecting https://www.ros.org/reps/rep-0103.html, i.e.,
// x forward
// y left
// z up
class OdometryToRos : public ProtoToRosConverter<Odometry2Proto, nav_msgs::Odometry> {
 public:
  bool protoToRos(Odometry2Proto::Reader reader, const ros::Time& ros_time,
                  nav_msgs::Odometry& ros_message) override;

  // Frame of the pose in outgoing message
  ISAAC_PARAM(std::string, pose_frame, "odom");
  // Frame of the twist in outgoing message
  ISAAC_PARAM(std::string, twist_frame, "base_footprint");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::OdometryToRos);
