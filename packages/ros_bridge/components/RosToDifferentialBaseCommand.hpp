/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "geometry_msgs/Twist.h"
#include "messages/state.capnp.h"
#include "packages/ros_bridge/components/RosToProtoConverter.hpp"

namespace isaac {
namespace ros_bridge {

// This codelet receives twist message from ROS and publishes it as a velocity command for a
// differential base robot.
class RosToDifferentialBaseCommand : public RosToProtoConverter<StateProto, geometry_msgs::Twist> {
 public:
  bool rosToProto(const geometry_msgs::Twist::ConstPtr& ros_message,
                  std::optional<ros::Time>& ros_time, StateProto::Builder builder,
                  std::vector<isaac::SharedBuffer>& buffers) override;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosToDifferentialBaseCommand);
