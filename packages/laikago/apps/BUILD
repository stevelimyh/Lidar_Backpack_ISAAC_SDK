"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_graph")

isaac_app(
    name = "laikago_joystick",
    data = [
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
    ],
    modules = [
        "laikago",
    ],
    tags = ["manual"],
)

isaac_graph(
    name = "laikago_hardware_subgraph",
    modules = [
        "laikago",
        "velodyne_lidar",
    ],
    graph = "laikago_hardware.subgraph.json",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "2d_laikago_subgraph",
    data = [
        ":laikago_hardware_subgraph",
        "//packages/navigation/apps:scan_flattener_subgraph",
    ],
    graph = "2d_laikago.subgraph.json",
    tags = ["manual"],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "laikago_navigate",
    data = [
        "2d_laikago_subgraph",
        "//apps/assets/maps",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_navigation_subgraph",
    ],
    tags = ["manual"],
)
