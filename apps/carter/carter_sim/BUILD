"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app")

isaac_app(
    name = "carter_sim",
    data = [
        "//apps/assets/maps",
        "//apps/carter/unreal_sim:2d_carter_sim_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
    ],
    modules = [
        "imu",
        "map",
        "navigation",
        "perception",
        "planner",
        "sensors:joystick",
        "viewers",
    ],
)

isaac_app(
    name = "carter_sim_joystick",
    data = [
        "//apps/carter/unreal_sim:carter_sim_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
    ],
    modules = [
        "flatsim",
        "navigation",
        "viewers",
    ],
)

isaac_app(
    name = "carter_sim_mapping",
    data = [
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/navigation/apps:local_map_subgraph",
        "//apps/carter/unreal_sim:2d_carter_sim_subgraph",
        "carter.lua",
    ],
    modules = [
        "lidar_slam:cartographer",
        "navigation",
        "planner",
        "viewers",
    ],
)

