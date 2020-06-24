"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library", "isaac_component")

isaac_component(
    name = "camera_image_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "camera_info_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "flatscan_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "odometry_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "goal_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "goal_to_ros_action",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "pose_message_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "poses_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_cc_library(
    name = "proto_to_ros_converter",
    hdrs = ["ProtoToRosConverter.hpp"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//engine/alice",
        "//engine/core",
        "//messages",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_node",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_differential_base_command",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//engine/gems/state:io",
        "//messages/state:differential_base",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_image",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//engine/alice/tools:websight",
        "//engine/gems/image",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_pose2_mean_and_covariance",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_poses",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_cc_library(
    name = "ros_to_proto_converter",
    hdrs = ["RosToProtoConverter.hpp"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//engine/alice",
        "//engine/core",
        "//messages",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)
