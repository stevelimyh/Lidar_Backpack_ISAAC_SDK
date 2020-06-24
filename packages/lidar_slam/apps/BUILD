"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_graph")

isaac_graph(
    name = "differential_base_gmapping_subgraph",
    data = [
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/record_replay/apps:record_subgraph",
    ],
    graph = "differential_base_gmapping.subgraph.json",
    modules = [
        "lidar_slam:g_mapping",
        "navigation",
        "sight",
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "differential_base_cartographer_subgraph",
    data = [
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/record_replay/apps:record_subgraph",
    ],
    graph = "differential_base_cartographer.subgraph.json",
    modules = [
        "lidar_slam:cartographer",
        "navigation",
        "sight",
    ],
    visibility = ["//visibility:public"],
)
