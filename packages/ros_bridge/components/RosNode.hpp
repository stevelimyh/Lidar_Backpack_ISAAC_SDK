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

#include "engine/alice/alice_codelet.hpp"
#include "packages/ros_bridge/gems/include_before_ros.hpp"

// Include ros.h after include_before_ros.hpp. See include_before_ros.hpp for details.
#include "ros/ros.h"

namespace isaac {
namespace ros_bridge {

// This codelet initializes a ROS node and ticks until roscore is up. Every Isaac application with
// ROS bridge needs to have one and only one node with a single component of this type.
// This codelet also provides
// 1. ros::NodeHandle, which can be used to initialize ROS message subscribers and publishers,
// 2. checkBeforeInterface() which should be used before carrying ROS operations.
class RosNode : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Returns null if RosNode is ready to publish, subscribe etc. Otherwise returns the error as
  // string, so that caller can print it.
  std::optional<std::string> checkBeforeInterface() const;

  // Returns a pointer to node_handle_
  ros::NodeHandle* get_node_handle() const;

  // Node name that will appear in ROS node diagram
  ISAAC_PARAM(std::string, ros_node_name, "isaac_bridge");

 private:
  // Do not deinitialize node_handle_. This codelet will stop ticking after ROS connection is
  // established, but other codelets will use this NodeHandle to communicate with ROS.
  std::unique_ptr<ros::NodeHandle> node_handle_;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosNode);
