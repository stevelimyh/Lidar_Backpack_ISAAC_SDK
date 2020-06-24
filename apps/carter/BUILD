"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_doc_dep", "isaac_subgraph")

isaac_subgraph(
    name = "carter_hardware_subgraph",
    modules = [
        "imu",
        "segway",
        "sight",
        "velodyne_lidar",
    ],
    subgraph = "carter_hardware.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "2d_carter_subgraph",
    data = [
        ":carter_hardware_subgraph",
        "//packages/navigation/apps:scan_flattener_subgraph",
    ],
    modules = [],
    subgraph = "2d_carter.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "carter",
    data = [
        ":2d_carter_subgraph",
        "//apps/assets/maps",
        "//apps/carter/robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "sensors:joystick",
        "viewers",
    ],
    script = "select_json.sh",
)

isaac_doc_dep(
    name = "doc",
    visibility = ["//visibility:public"],
    deps = ["//apps/carter/gmapping/doc"],
)
