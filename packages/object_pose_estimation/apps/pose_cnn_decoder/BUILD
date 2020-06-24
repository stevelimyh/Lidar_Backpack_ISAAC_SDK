"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app", "isaac_subgraph")

filegroup(
    name = "config",
    srcs = [
        "detection_pose_estimation_cnn_inference_dolly.config.json",
        "detection_pose_estimation_cnn_inference_kltSmall.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "simulation_subgraph",
    modules = [
        "ml",
        "navsim",
        "viewers",
    ],
    subgraph = "pose_estimation_sim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "pose_estimation_cnn_subgraph",
    data = [
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "perception",
    ],
    subgraph = "pose_estimation_cnn.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "detection_pose_estimation_cnn_inference_app",
    srcs = ["detection_pose_estimation_cnn_inference_app.py"],
    data = [
        "config",
        "detection_pose_estimation_cnn_inference",
        "simulation_subgraph",
        "//apps:py_init",
        "//packages/record_replay/apps:replay_subgraph",
        "@industrial_dolly_pose_estimation_data",
        "@sortbot_pose_estimation_data",
    ],
    modules = [
        "message_generators",
        "ml",
        "object_pose_estimation",
        "realsense",
        "sight",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_subgraph(
    name = "detection_pose_estimation_cnn_inference",
    data = [
        "config",
        "pose_estimation_cnn_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@sortbot_pose_estimation_models",
    ],
    modules = [
        "utils",
        "sight",
        "viewers",
    ],
    subgraph = "detection_pose_estimation_cnn_inference.subgraph.json",
    visibility = ["//visibility:public"],
)
