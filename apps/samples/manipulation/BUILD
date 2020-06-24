"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_jupyter_app", "isaac_py_app", "isaac_subgraph")

isaac_jupyter_app(
    name = "simple_joint_control",
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "kinova_jaco",
        "sight",
    ],
    notebook = "simple_joint_control.ipynb",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "shuffle_box_behavior_subgraph",
    modules = [
        "behavior_tree",
        "composite",
    ],
    subgraph = "shuffle_box_behavior.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "shuffle_box",
    srcs = ["shuffle_box.py"],
    data = [
        ":config",
        ":shuffle_box_behavior_subgraph",
        "//apps/assets/kinematic_trees",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "behavior_tree",
        "ml",
        "object_pose_estimation",
        "planner",
        "sight",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

filegroup(
    name = "config",
    srcs = [
        "shuffle_box_detection_pose_estimation.config.json",
    ],
    visibility = ["//visibility:public"],
)