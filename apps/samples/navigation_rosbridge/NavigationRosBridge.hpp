/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

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
#include "messages/differential_base.capnp.h"

namespace isaac {
namespace rosbridge {

// This codelet represents a basic bridge to ROS for communicating navigation events.
// It receives Pose2D messages from ROS and emits goal protos
// It will also check the current pose of the robot and publish Pose2D messages to ROS
class NavigationRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  NavigationRosBridge();
  virtual ~NavigationRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  // Output goal for the robot
  ISAAC_PROTO_TX(Goal2Proto, goal);

  // Output pose where the estimated pose of the robot is written
  ISAAC_POSE2(world, robot);

  // ROS publisher queue depth
  ISAAC_PARAM(int, publisher_queue_size, 1000);
  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, publisher_channel_name, "isaac_navigation2D_status");
  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, subscriber_channel_name, "isaac_navigation2D_request");

 private:
  // Hide the ROS implementation details
  struct RosNavigationData;
  std::unique_ptr<RosNavigationData> nav_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::NavigationRosBridge);
