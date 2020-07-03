"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app")

isaac_py_app(
    name = "navsim_navigate",
    srcs = [
        "navsim_navigate.py",
    ],
    data = [
        "navsim_navigate.app.json",
        "//apps/assets/maps",
        "//packages/behavior_tree/apps:missions",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "behavior_tree",
        "map",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_app(
    name = "navsim_gmapping",
    data = [
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "navsim_record",
    srcs = [
        "navsim_record.py",
    ],
    data = [
        "navsim_record.app.json",
        "//apps/assets/maps",
        "//packages/behavior_tree/apps:missions",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
        "//packages/record_replay/apps:record_subgraph",
    ],
    modules = [
        "behavior_tree",
        "map",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_app(
    name = "navsim_replay_gmap",
    data = [
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/navsim/robots",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_sub_gmap",
    data = [
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/navsim/robots",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_manual",
    data = [
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/record_replay/apps:record_subgraph",
    ],
     visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_manual_gmap",
    data = [
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "viewers",
    ],

    script = "select_json_gmapping.sh",
     visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_camera_test",
    data = [
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/record_replay/apps:record_subgraph",
    ],
     modules = [
        "viewers",
        "perception",
    ],
     visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_manual_carto",
    data = [
        "carter.lua",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/lidar_slam/apps:differential_base_cartographer_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "viewers",
    ],

    script = "select_json_gmapping.sh",
     visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_sub_carto",
    data = [
        "carter.lua",
        "//packages/lidar_slam/apps:differential_base_cartographer_subgraph",
        "//packages/navsim/robots",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_replay_carto",
    data = [
        "carter.lua",
        "//packages/lidar_slam/apps:differential_base_cartographer_subgraph",
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/navsim/robots",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)