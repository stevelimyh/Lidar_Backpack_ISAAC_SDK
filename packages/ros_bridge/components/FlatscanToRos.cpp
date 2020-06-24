/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "FlatscanToRos.hpp"

namespace isaac {
namespace ros_bridge {

bool FlatscanToRos::protoToRos(FlatscanProto::Reader reader, const ros::Time& ros_time,
                               sensor_msgs::LaserScan& ros_message) {
  // Read data from Isaac type
  auto msg_ranges = reader.getRanges();
  auto msg_angles = reader.getAngles();
  if (msg_ranges.size() != msg_angles.size()) {
    return false;
  }
  const size_t num_beams = msg_ranges.size();
  if (num_beams == 0) {
    return false;
  }

  // Populate data for ROS type
  ros_message.header.stamp = ros_time;
  ros_message.header.frame_id = get_frame_id();
  ros_message.ranges.resize(num_beams);
  ros_message.angle_min = msg_angles[0];
  ros_message.angle_max = msg_angles[num_beams - 1];
  ros_message.angle_increment = (ros_message.angle_max - ros_message.angle_min) / (num_beams - 1);
  for (size_t i = 0; i < num_beams; i++) {
    ros_message.ranges[i] = msg_ranges[i];
    if (!IsAlmostEqualRelative(msg_angles[i],
                               ros_message.angle_min + i * ros_message.angle_increment)) {
      return false;
    }
  }
  ros_message.range_min = reader.getInvalidRangeThreshold();
  ros_message.range_max = reader.getOutOfRangeThreshold();
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
