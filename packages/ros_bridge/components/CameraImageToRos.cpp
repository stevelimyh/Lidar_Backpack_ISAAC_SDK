/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CameraImageToRos.hpp"

#include <vector>

#include "messages/camera.hpp"

namespace isaac {
namespace ros_bridge {

bool CameraImageToRos::protoToRos(ColorCameraProto::Reader reader, const ros::Time& ros_time,
                                  sensor_msgs::Image& ros_message) {
  // Populate data for ROS type
  // Reading from Isaac buffer.
  ImageProto::Reader read_image = reader.getImage();
  const size_t cols = reader.getImage().getCols();
  const size_t rows = reader.getImage().getRows();
  const size_t channels = reader.getImage().getChannels();

  ImageConstView3ub image;
  if (!FromProto(read_image, rx_proto().buffers(), image)) {
    reportFailure("Reading input image from proto buffer failed.");
    return false;
  }

  ros_message.header.stamp = ros_time;
  ros_message.header.frame_id = get_frame_id();
  ros_message.width = cols;
  ros_message.height = rows;
  if (reader.getColorSpace() == ColorCameraProto::ColorSpace::RGB) {
    ros_message.encoding = "rgb8";
  } else {
    reportFailure("Unsupported image format");
    return false;
  }

  ros_message.step = cols * channels;
  unsigned const char* image_ptr = static_cast<unsigned const char*>(image.element_wise_begin());
  ros_message.data = std::vector<unsigned char>(image_ptr, image_ptr + ros_message.step * rows);
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
