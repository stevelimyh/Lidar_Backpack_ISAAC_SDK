/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosNode.hpp"

#include <memory>
#include <string>
#include <vector>

namespace isaac {
namespace ros_bridge {

namespace {
// Print a message every second if roscore is not up
constexpr double kWarningPeriod = 1.0;
}  // namespace

void RosNode::start() {
  node_handle_ = nullptr;

  // Make sure we don't have more than one instances of this codelet
  std::vector<RosNode*> ros_nodes = node()->app()->findComponents<RosNode>();
  ASSERT(ros_nodes.size() == 1,
         "An Isaac application with ROS bridge needs to have one and only one RosNode component. "
         "This application has %zu.",
         ros_nodes.size());

  if (ros::isInitialized()) {
    reportFailure("ros::init should be called by RosNode");
    return;
  }

  // Make sure to disable the Sigint handler and if any arguments are needed add them to the args
  // string.
  ros::M_string args;
  ros::init(args, get_ros_node_name(), ros::init_options::NoSigintHandler);

  tickPeriodically();
}

void RosNode::tick() {
  // roscore needs to be started externally.
  if (!ros::master::check()) {
    if (stopwatch().read() > kWarningPeriod || isFirstTick()) {
      stopwatch().start();
      LOG_ERROR("ROS is not running. Please start roscore.");
    }
    return;
  }

  node_handle_ = std::make_unique<ros::NodeHandle>();

  // roscore is up. Our ros node is initialized. RosInterface is created.
  LOG_INFO("Initialization is complete");
  reportSuccess();
}

ros::NodeHandle* RosNode::get_node_handle() const {
  ASSERT(node_handle_, "Can't get ros::NodeHandle");
  return node_handle_.get();
}

// Returns null if Ros is up. Returns error description if not.
std::optional<std::string> RosNode::checkBeforeInterface() const {
  if (!node_handle_) {
    return "RosNode is not ready. Please check your behavior tree.";
  }
  if (!ros::ok()) {
    return "ros::ok() returned false.";
  }
  if (!ros::master::check()) {
    return "Lost connection with Ros master. Is roscore still up?";
  }
  return std::nullopt;
}

}  // namespace ros_bridge
}  // namespace isaac
