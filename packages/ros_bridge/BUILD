"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module")

isaac_cc_module(
    name = "ros_bridge",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/components:camera_image_to_ros",
        "//packages/ros_bridge/components:camera_info_to_ros",
        "//packages/ros_bridge/components:flatscan_to_ros",
        "//packages/ros_bridge/components:goal_to_ros",
        "//packages/ros_bridge/components:goal_to_ros_action",
        "//packages/ros_bridge/components:odometry_to_ros",
        "//packages/ros_bridge/components:pose_message_to_ros",
        "//packages/ros_bridge/components:poses_to_ros",
        "//packages/ros_bridge/components:ros_node",
        "//packages/ros_bridge/components:ros_to_differential_base_command",
        "//packages/ros_bridge/components:ros_to_image",
        "//packages/ros_bridge/components:ros_to_pose2_mean_and_covariance",
        "//packages/ros_bridge/components:ros_to_poses",
    ],
)
