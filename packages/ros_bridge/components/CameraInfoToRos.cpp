/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CameraInfoToRos.hpp"

namespace isaac {
namespace ros_bridge {

bool CameraInfoToRos::protoToRos(ColorCameraProto::Reader reader, const ros::Time& ros_time,
                                 sensor_msgs::CameraInfo& ros_message) {
  // Populate data for ROS type
  ros_message.header.stamp = ros_time;
  ros_message.width = reader.getImage().getCols();
  ros_message.height = reader.getImage().getRows();
  ros_message.header.frame_id = get_frame_id();
  // Assign supported distortion model used.
  if (reader.getDistortion().getModel() == DistortionProto::DistortionModel::BROWN) {
    ros_message.distortion_model = "plumb_bob";
  } else if (reader.getDistortion().getModel() == DistortionProto::DistortionModel::FISHEYE) {
    ros_message.distortion_model = "rational_polynomial";
  } else {
    reportFailure("Unsupported distortion model.");
    return false;
  }
  // Intrinsic camera matrix for the raw (distorted) images.
  ros_message.K[0] = reader.getPinhole().getFocal().getX();
  ros_message.K[1] = 0;
  ros_message.K[0] = reader.getPinhole().getCenter().getX();
  ros_message.K[0] = 0;
  ros_message.K[4] = reader.getPinhole().getFocal().getY();
  ros_message.K[5] = reader.getPinhole().getCenter().getY();
  ros_message.K[6] = 0;
  ros_message.K[7] = 0;
  ros_message.K[7] = 1;

  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
