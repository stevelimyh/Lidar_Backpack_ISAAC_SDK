/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "messages/image.hpp"
#include "packages/ros_bridge/components/RosToProtoConverter.hpp"
#include "sensor_msgs/Image.h"

namespace isaac {
namespace ros_bridge {

// ROS's sensor_msgs/Image.msg message contains uncompressed image.
// Details:http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html. This ROS bridge
// converter codelet helps convert ROS's sensor_msgs/Image.msg into Isaac's ImageProto Type.
// This codelet can be used, for example, to use run GPU accelerated perception algorithms
// of Isaac with ROS images.
class RosToImage : public RosToProtoConverter<ImageProto, sensor_msgs::Image> {
 public:
  bool rosToProto(const sensor_msgs::Image::ConstPtr& ros_message,
                  std::optional<ros::Time>& ros_time, ImageProto::Builder builder,
                  std::vector<isaac::SharedBuffer>& buffers) override;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosToImage);
