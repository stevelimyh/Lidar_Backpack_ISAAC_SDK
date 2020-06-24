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
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/components/TimeSynchronizer.hpp"
#include "packages/ros_bridge/components/RosNode.hpp"
#include "ros/callback_queue.h"

namespace isaac {
namespace ros_bridge {

// Base class for ros_bridge converters that subscribe to ROS messages to publish Isaac protos.
// All derived class needs to do is to override one of the rosToProto() methods.
// channel_name has to be specified in the configuration.
// For an example, please see RosToDifferentialBaseCommand.
// Do not ISAAC_ALICE_REGISTER_CODELET this base class. Instead, register derived classes.
template <typename IsaacProtoType, typename RosMsgType>
class RosToProtoConverter : public alice::Codelet {
 public:
  void start() override {
    // Get RosNode pointer
    const std::string ros_node_name = get_ros_node();
    ros_node_ = node()->app()->getNodeComponentOrNull<RosNode>(ros_node_name);
    if (!ros_node_) {
      reportFailure("No RosNode component named '%s'", ros_node_name.c_str());
      return;
    }
    time_synchronizer_ =
        node()->app()->getNodeComponentOrNull<alice::TimeSynchronizer>(ros_node_name);
    if (!time_synchronizer_) {
      reportFailure("No TimeSynchronizer component in '%s' node", ros_node_name.c_str());
      return;
    }
    // Check with RosNode
    if (auto maybe_error = ros_node_->checkBeforeInterface()) {
      reportFailure(maybe_error->c_str());
      return;
    }
    // Create subscriber
    try {
      ros::NodeHandle* node_handle = ros_node_->get_node_handle();
      ros::SubscribeOptions options = ros::SubscribeOptions::create<RosMsgType>(
          get_channel_name(), get_queue_size(),
          boost::bind(&RosToProtoConverter::convertAndPublish, this, _1), ros::VoidPtr(),
          &callback_queue_);
      subscriber_ = node_handle->subscribe(options);
      if (!subscriber_) {
        reportFailure("ros::NodeHandle::subscribe() failed.");
        return;
      }
    } catch (const ros::Exception& exception) {
      reportFailure("Caught ros::Exception: %s", exception.what());
      return;
    }
    // Tick periodically
    tickPeriodically();
  }

  void tick() override {
    // Check with RosNode
    ASSERT(ros_node_, "Logic error");
    if (auto maybe_error = ros_node_->checkBeforeInterface()) {
      reportFailure(maybe_error->c_str());
      return;
    }
    // Pump the queue. This will invoke the callback function.
    callback_queue_.callAvailable();
  }

  void stop() override {
    // ros_node will be null if we failed to start
    if (!ros_node_) {
      return;
    }
    // Shut down the Subscriber
    subscriber_.shutdown();
  }

  // This function is called every tick for each message received
  void convertAndPublish(const typename RosMsgType::ConstPtr& ros_message) {
    // Check with RosNode
    if (auto maybe_error = ros_node_->checkBeforeInterface()) {
      reportFailure(maybe_error->c_str());
      return;
    }
    // Use derived class' rosToProto to convert ROS message to Isaac proto
    std::optional<ros::Time> ros_time = std::nullopt;
    if (!rosToProto(ros_message, ros_time, tx_proto().initProto(), tx_proto().buffers())) {
      return;
    }
    // Publish
    if (ros_time) {
      const int64_t sync_time = SecondsToNano(ros_time->toSec());
      const int64_t app_time = time_synchronizer_->syncToAppTime(sync_time);
      tx_proto().publish(app_time);
    } else {
      tx_proto().publish();
    }
  }

  // Populates Isaac proto using ROS message. Returns false if it fails.
  // One of these rosToProto functions need to be overridden by derived classes.
  // Override the first one if buffers are needed. Override the second otherwise.
  virtual bool rosToProto(const typename RosMsgType::ConstPtr& ros_message,
                          std::optional<ros::Time>& ros_time,
                          typename IsaacProtoType::Builder builder,
                          std::vector<isaac::SharedBuffer>& buffers) {
    return rosToProto(ros_message, ros_time, builder);
  }
  virtual bool rosToProto(const typename RosMsgType::ConstPtr& ros_message,
                          std::optional<ros::Time>& ros_time,
                          typename IsaacProtoType::Builder builder) {
    return false;
  }

  // Output Isaac data, which will be published after receiving and converting ROS message
  ISAAC_PROTO_TX(IsaacProtoType, proto);

  // Name of the Isaac node with RosNode component
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, ros_node, "ros_node");
  // ROS subscriber queue depth
  // Needs to be set before the application starts.
  ISAAC_PARAM(int, queue_size, 1000);
  // ROS channel where messages will be received from ROS
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, channel_name);

 private:
  ros::Subscriber subscriber_;
  ros::CallbackQueue callback_queue_;
  alice::TimeSynchronizer* time_synchronizer_;
  RosNode* ros_node_;
};

}  // namespace ros_bridge
}  // namespace isaac
