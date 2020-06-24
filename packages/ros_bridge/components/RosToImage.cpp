/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToImage.hpp"

#include <utility>
#include <vector>

#include "engine/gems/image/utils.hpp"

namespace isaac {
namespace ros_bridge {

bool RosToImage::rosToProto(const sensor_msgs::Image::ConstPtr& ros_message,
                            std::optional<ros::Time>& ros_time, ImageProto::Builder builder,
                            std::vector<isaac::SharedBuffer>& buffers) {
  ros_time = ros_message->header.stamp;
  // This converter is currently only supporting rgb8 encoding.
  if (ros_message->encoding != "rgb8") {
    reportFailure("Unsupported image format: %s", ros_message->encoding.c_str());
    return false;
  }
  ImageConstView3ub rgb_image = CreateImageView<uint8_t, 3>(
      static_cast<const uint8_t*>(&ros_message->data[0]), ros_message->height, ros_message->width);
  Image3ub color_image(rgb_image.dimensions());
  Copy(rgb_image, color_image);
  show("image", [&](sight::Sop& sop) { sop.add(rgb_image); });

  builder.setRows(ros_message->height);
  builder.setCols(ros_message->width);
  builder.setDataBufferIndex(0);
  ToProto(std::move(color_image), builder, buffers);

  return true;
}

}  // namespace ros_bridge
}  // namespace isaac

