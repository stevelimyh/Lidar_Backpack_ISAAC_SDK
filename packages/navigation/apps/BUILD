"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "differential_base_navigation_subgraph",
    data = [
        ":differential_base_control_subgraph",
        ":differential_base_imu_odometry_subgraph",
        ":go_to_subgraph",
        ":local_map_subgraph",
        ":planner_subgraph",
        "//packages/flatscan_localization/apps:localization_subgraph",
    ],
    modules = [
        "map",
        "navigation",
        "planner",
        "sight",
    ],
    subgraph = "differential_base_navigation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "holonomic_base_navigation_subgraph",
    data = [
        ":go_to_subgraph",
        ":holonomic_base_control_subgraph",
        ":holonomic_base_imu_odometry_subgraph",
        ":local_map_subgraph",
        ":planner_subgraph",
        "//packages/flatscan_localization/apps:localization_subgraph",
    ],
    modules = [
        "map",
        "navigation",
        "planner",
        "sight",
    ],
    subgraph = "holonomic_base_navigation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "differential_base_control_subgraph",
    modules = [
        "behavior_tree",
        "message_generators",
        "navigation",
        "planner",
    ],
    subgraph = "differential_base_control.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "holonomic_base_control_subgraph",
    modules = [
        "behavior_tree",
        "message_generators",
        "navigation",
        "planner",
    ],
    subgraph = "holonomic_base_control.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "differential_base_commander_subgraph",
    modules = [
        "navigation",
        "sensors:joystick",
    ],
    subgraph = "differential_base_commander.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "holonomic_base_commander_subgraph",
    modules = [
        "navigation",
        "sensors:joystick",
    ],
    subgraph = "holonomic_base_commander.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "goal_generators_subgraph",
    modules = [
        "behavior_tree",
        "navigation",
    ],
    subgraph = "goal_generators.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "go_to_subgraph",
    modules = [
        "behavior_tree",
        "navigation",
        "viewers",
    ],
    subgraph = "go_to.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "differential_base_imu_odometry_subgraph",
    modules = [
        "imu",
        "navigation",
    ],
    subgraph = "differential_base_imu_odometry.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "holonomic_base_imu_odometry_subgraph",
    modules = [
        "imu",
        "navigation",
    ],
    subgraph = "holonomic_base_imu_odometry.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "scan_flattener_subgraph",
    modules = [
        "perception",
    ],
    subgraph = "scan_flattener.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "planner_subgraph",
    modules = [
        "navigation",
        "planner",
    ],
    subgraph = "planner.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "local_map_subgraph",
    modules = [
        "navigation",
    ],
    subgraph = "local_map.subgraph.json",
    visibility = ["//visibility:public"],
)
