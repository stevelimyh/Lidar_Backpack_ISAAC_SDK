"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_subgraph(
    name = "cart_delivery_subgraph",
    data = [
        "navigation.config.json",
        "pose2_planner.config.json",
        ":detection_pose_estimation.config.json",
        "//apps/assets/maps",
        "//packages/behavior_tree/apps:missions",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@pose2_grid_graphs_factory",
    ],
    modules = [
        "behavior_tree",
        "message_generators",
        "navigation",
        "utils",
    ],
    subgraph = "cart_delivery.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navigate",
    data = [
        ":navigation.config.json",
        ":pose2_planner.config.json",
        ":robot_with_dolly.config.json",
        "//apps/assets/maps",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
        "@pose2_grid_graphs_factory",
    ],
    modules = [
        "behavior_tree",
        "map",
        "message_generators",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "perception",
    data = [
        ":detection_pose_estimation.config.json",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "gmapping",
    data = [
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "cart_delivery_json",
    app_json_file = "cart_delivery.app.json",
    data = [
        ":cart_delivery_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "behavior_tree",
        "map",
    ],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "cart_delivery",
    srcs = ["cart_delivery.py"],
    data = [
        ":cart_delivery_json",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
    ],
)
