"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "waypoint_from_slack",
    srcs = ["WaypointFromSlack.cpp"],
    hdrs = ["WaypointFromSlack.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/behavior_tree/components",
        "//engine/gems/algorithm:string_utils",
        "//packages/map:public_headers",
    ],
)

isaac_app(
    name = "slackbot",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots:robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "//apps/carter/slackbot:waypoint_from_slack",
        "communications",
        "map",
        "navigation",
        "perception",
        "imu",
        "planner",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
        "viewers",
    ],
)
