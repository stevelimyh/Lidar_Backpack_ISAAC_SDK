"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph", "isaac_py_app")

isaac_subgraph(
    name = "detect_net_inference",
    data = [
        "@industrial_dolly_pose_estimation_cnn_model",
        "@resnet_object_detection_model",
        "@sortbot_pose_estimation_models",
    ],
    modules = [
        "detect_net",
        "ml",
        "perception",
        "sight",
        "viewers",
    ],
    subgraph = "detect_net_inference.subgraph.json",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "config",
    srcs = [
        "detect_net_industrial_dolly.config.json",
        "detect_net_industrial_dolly_fof.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "detect_net_inference_app",
    srcs = ["detect_net_inference_app.py"],
    data = [
        "config",
        "detect_net_inference",
        "//apps:py_init",
        "//packages/navsim/apps:navsim_training_subgraph",
        "//packages/record_replay/apps:replay_subgraph",
        "@industrial_dolly_pose_estimation_data",
        "@sortbot_pose_estimation_data",
    ],
    modules = [
        "message_generators",
        "realsense",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)