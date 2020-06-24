"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "camera_to_ros",
    data = [
        ":camera_to_ros_subgraph",
    ],
    modules = [
        "sensors:v4l2_camera",
    ],
    tags = ["manual"],
)

isaac_subgraph(
    name = "camera_to_ros_subgraph",
    modules = [
        "behavior_tree",
        "ros_bridge",
        "viewers",
    ],
    subgraph = "camera_to_ros.subgraph.json",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "ros_to_navigation_config",
    data = [
        "ros_to_navigation_turtlebot3_waffle_pi.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "ros_to_navigation_subgraph",
    data = [
        ":ros_to_navigation_bridge_subgraph",
        ":ros_to_navigation_config",
        "//apps/assets/maps",
        "//packages/ros_bridge/maps",
    ],
    modules = [
        "navigation",
        "planner",
        "viewers",
    ],
    subgraph = "ros_to_navigation.subgraph.json",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "ros_to_navigation_bridge_subgraph",
    modules = [
        "behavior_tree",
        "ros_bridge",
    ],
    subgraph = "ros_to_navigation_bridge.subgraph.json",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "ros_to_navigation_flatsim",
    data = [
        ":ros_to_navigation_subgraph",
        "//packages/flatsim/apps:2d_differential_base_simulation_subgraph",
    ],
    modules = [
        "map",
    ],
    tags = ["manual"],
)

isaac_app(
    name = "ros_to_navigation_unity3d",
    data = [
        ":ros_to_navigation_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/scenarios",
    ],
    modules = [
        "map",
    ],
    tags = ["manual"],
)
