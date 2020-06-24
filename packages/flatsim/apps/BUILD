"""
Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("//engine/build:isaac.bzl", "isaac_pkg", "isaac_graph", "isaac_subgraph", "isaac_py_app")

isaac_graph(name = "demo_1", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_2", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_3", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_4", modules = ["flatsim", "map", "navigation"],
            data = ["//apps/assets/maps"])
isaac_graph(name = "demo_5", modules = ["flatsim", "map", "navigation"],
            data = ["//apps/assets/maps"])

filegroup(
    name = "maps",
    visibility = ["//visibility:public"],
    data = [
        "demo_1",
        "demo_2",
        "demo_3",
        "demo_4",
        "demo_5",
    ],
)

isaac_py_app(
    name = "flatsim",
    srcs = [
        "__init__.py",
        "flatsim.py"
    ],
    data = [
        "flatsim_subgraph",
        "maps",
        "//packages:py_init",
        "//packages/flatsim:py_init",
    ],
    modules=[
        'flatsim',
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_graph(
    name = "2d_differential_base_simulation_subgraph",
    modules = ["//packages/flatsim"],
    graph = "2d_differential_base_simulation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "flatsim_subgraph",
    data = [
        "2d_differential_base_simulation_subgraph",
        "differential_base_navigation_slam_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
    ],
    graph = "flatsim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "differential_base_navigation_slam_subgraph",
    data = [
        "//packages/navigation/apps:differential_base_control_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/navigation/apps:go_to_subgraph",
        "//packages/navigation/apps:local_map_subgraph",
        "//packages/navigation/apps:planner_subgraph",
        "//packages/flatscan_localization/apps:localization_subgraph",
    ],
    modules = [
        "lidar_slam:g_mapping",
        "map",
        "navigation",
        "planner",
        "sight",
    ],
    subgraph = "differential_base_navigation_slam.subgraph.json",
    visibility = ["//visibility:public"],
)
