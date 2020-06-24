"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_subgraph", "isaac_app", "isaac_graph")

isaac_subgraph(
    name = "multi_joint_lqr_control_subgraph",
    modules = [
        "map",
        "planner",
    ],
    subgraph = "multi_joint_lqr_control.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "pose2_graph_builder",
    data = [
        "pose2_graph_builder_subgraph",
        "//apps/assets/maps",
    ],
    modules = [
        "map",
        "planner",
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "pose2_graph_builder_subgraph",
    modules = [
        "planner",
    ],
    graph = "pose2_graph_builder.subgraph.json",
    visibility = ["//visibility:public"],
)