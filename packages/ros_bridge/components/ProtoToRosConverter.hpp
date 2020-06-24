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

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/components/TimeSynchronizer.hpp"
#include "packages/ros_bridge/components/RosNode.hpp"

namespace isaac {
namespace ros_bridge {

// Base class for ros_bridge converters that publish ROS messages by reading Isaac protos.
// All derived class needs to do is to define protoToRos().
// channel_name has to be specified in the configuration.
// For an example, please see FlatscanToRos.
// Do not ISAAC_ALICE_REGISTER_CODELET this base class. Instead, register derived classes.
// Set Latch=true to make the last published mesage available to future subscribers.
// This is needed, for example, when we send initial pose (once) to ROS. Otherwise amcl will
// likely never receive it.
template <typename IsaacProtoType, typename RosMsgType, bool Latch = false>
class ProtoToRosConverter : public alice::Codelet {
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
    // Advertise publisher
    try {
      ros::NodeHandle* node_handle = ros_node_->get_node_handle();
      publisher_ = node_handle->advertise<RosMsgType>(get_channel_name(), get_queue_size(), Latch);
      if (!publisher_) {
        reportFailure("ros::NodeHandle::advertise() failed.");
        return;
      }
    } catch (const ros::Exception& exception) {
      reportFailure("Caught ros::Exception: %s", exception.what());
      return;
    }
    // Tick at every input message
    tickOnMessage(rx_proto());
  }

  void tick() override {
    // Check with RosNode
    ASSERT(ros_node_, "Logic error");
    if (auto maybe_error = ros_node_->checkBeforeInterface()) {
      reportFailure(maybe_error->c_str());
      return;
    }
    // Get the time stamp
    const ros::Time ros_time =
        ros::Time(ToSeconds(time_synchronizer_->appToSyncTime(rx_proto().acqtime())));
    // Use derived class' protoToRos to convert Isaac proto to ROS message
    RosMsgType ros_message;
    if (!protoToRos(rx_proto().getProto(), ros_time, ros_message)) {
      return;
    }
    // Publish ROS message
    publisher_.publish(ros_message);
  }

  void stop() override {
    // ros_node will be null if we failed to start
    if (!ros_node_) {
      return;
    }
    // Shut down the publisher
    publisher_.shutdown();
  }

  // Populates ROS message using Isaac proto. Returns false if it fails.
  // This function needs to be overridden by derived classes.
  virtual bool protoToRos(typename IsaacProtoType::Reader reader, const ros::Time& ros_time,
                          RosMsgType& ros_message) = 0;

  // Input Isaac data, which will be published to ROS after conversion
  ISAAC_PROTO_RX(IsaacProtoType, proto);

  // Name of the Isaac node with RosNode component
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, ros_node, "ros_node");
  // ROS publisher queue depth
  // Needs to be set before the application starts.
  ISAAC_PARAM(int, queue_size, 1000);
  // ROS channel where messages will be broadcasted for ROS
  // Needs to be set before the application starts.
  ISAAC_PARAM(std::string, channel_name);

 private:
  alice::TimeSynchronizer* time_synchronizer_;
  RosNode* ros_node_;
  ros::Publisher publisher_;
};

}  // namespace ros_bridge
}  // namespace isaac
