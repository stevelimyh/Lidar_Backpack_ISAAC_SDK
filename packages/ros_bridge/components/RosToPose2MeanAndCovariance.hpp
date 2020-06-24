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
#include "messages/math.capnp.h"
#include "packages/ros_bridge/components/RosToProtoConverter.hpp"

namespace isaac {
namespace ros_bridge {

// This codelet receives PoseWithCovarianceStamped message from ROS and publishes it as a 2D pose
// mean and covariance matrix
class RosToPose2MeanAndCovariance
    : public RosToProtoConverter<Pose2MeanAndCovariance, geometry_msgs::PoseWithCovarianceStamped> {
 public:
  bool rosToProto(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_message,
                  std::optional<ros::Time>& ros_time,
                  Pose2MeanAndCovariance::Builder builder) override;

  // Frame which incoming ros message is with respect to
  ISAAC_PARAM(std::string, ros_frame, "ros_map");
  // Frame which outgoing isaac message is with respect to
  ISAAC_PARAM(std::string, reference_frame, "world");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosToPose2MeanAndCovariance);
