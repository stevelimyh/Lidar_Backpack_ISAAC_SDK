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

#include "messages/flatscan.capnp.h"
#include "packages/ros_bridge/components/ProtoToRosConverter.hpp"
#include "sensor_msgs/LaserScan.h"

namespace isaac {
namespace ros_bridge {

// This codelet receives flatscan data within Isaac application and publishes it to ROS.
class FlatscanToRos : public ProtoToRosConverter<FlatscanProto, sensor_msgs::LaserScan> {
 public:
  bool protoToRos(FlatscanProto::Reader reader, const ros::Time& ros_time,
                  sensor_msgs::LaserScan& ros_message) override;

  // Name of the frame to be used in outgoing message
  ISAAC_PARAM(std::string, frame_id, "base_scan");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::FlatscanToRos);
