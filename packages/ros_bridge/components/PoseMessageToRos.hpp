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

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "messages/alice.capnp.h"
#include "packages/ros_bridge/components/ProtoToRosConverter.hpp"

namespace isaac {
namespace ros_bridge {

// This codelet receives pose as a message within Isaac application and publishes it to ROS.
// It can be used, for example, to send initial pose to ROS navigation stack when global
// localization is needed.
class PoseMessageToRos
    : public ProtoToRosConverter<PoseTreeEdgeProto, geometry_msgs::PoseWithCovarianceStamped,
                                 true> {
 public:
  bool protoToRos(PoseTreeEdgeProto::Reader reader, const ros::Time& ros_time,
                  geometry_msgs::PoseWithCovarianceStamped& ros_message) override;

  // If true, success will be reported after the first message publication. So, only one message
  // will be published to ROS. This is useful, for example, when initial pose is sent to ROS. It
  // needs to be done only once.
  ISAAC_PARAM(bool, report_success, false);
  // Name of the frame to be used in outgoing message
  ISAAC_PARAM(std::string, frame_id, "map");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::PoseMessageToRos);
