/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToPose2MeanAndCovariance.hpp"

#include "messages/math.hpp"

namespace isaac {
namespace ros_bridge {

namespace {

// Converts 6x6 covariance matrix in ROS format to 3x3 covariance matrix in Isaac format.
// Input covariance matrix is Row-Major and has order: (x, y, z, roll, pitch, yaw)
// Output covariance matrix has order: (x, y, yaw)
Matrix3d RosCovariance3ToIsaacCovariance2(const boost::array<double, 36>& ros_covariance) {
  Matrix3d isaac_covariance;
  isaac_covariance(0, 0) = ros_covariance[0 * 6 + 0];
  isaac_covariance(0, 1) = ros_covariance[0 * 6 + 1];
  isaac_covariance(0, 2) = ros_covariance[0 * 6 + 5];
  isaac_covariance(1, 0) = ros_covariance[1 * 6 + 0];
  isaac_covariance(1, 1) = ros_covariance[1 * 6 + 1];
  isaac_covariance(1, 2) = ros_covariance[1 * 6 + 5];
  isaac_covariance(2, 0) = ros_covariance[5 * 6 + 0];
  isaac_covariance(2, 1) = ros_covariance[5 * 6 + 1];
  isaac_covariance(2, 2) = ros_covariance[5 * 6 + 5];
  return isaac_covariance;
}

}  // namespace

bool RosToPose2MeanAndCovariance::rosToProto(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_message,
    std::optional<ros::Time>& ros_time, Pose2MeanAndCovariance::Builder builder) {
  // Read from Pose Tree
  const auto maybe_isaac_T_ros =
      node()->pose().tryGetPose2XY(get_reference_frame(), get_ros_frame(), getTickTime());
  if (!maybe_isaac_T_ros) {
    return false;
  }

  // Read from ros message
  const geometry_msgs::PoseWithCovariance pose_with_covariance = ros_message->pose;
  const boost::array<double, 36>& ros_covariance = pose_with_covariance.covariance;
  const geometry_msgs::Pose& pose = pose_with_covariance.pose;
  const geometry_msgs::Point& position = pose.position;
  const geometry_msgs::Quaternion& rotation = pose.orientation;
  ros_time = ros_message->header.stamp;

  // Create Isaac type
  const Pose2d ros_T_mean =
      Pose3d{SO3d::FromQuaternion(Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)),
             Vector3d{position.x, position.y, position.z}}
          .toPose2XY();
  const Pose2d reference_T_mean = *maybe_isaac_T_ros * ros_T_mean;
  const Matrix3d isaac_covariance = RosCovariance3ToIsaacCovariance2(ros_covariance);

  // Populate Isaac message
  ToProto(reference_T_mean, builder.initMean());
  ToProto(isaac_covariance, builder.initCovariance());
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
