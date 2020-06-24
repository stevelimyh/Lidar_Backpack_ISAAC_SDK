"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_pkg", "isaac_subgraph")

py_binary(
    name = "autoencoder_training",
    srcs = [
        "__init__.py",
        "autoencoder_training.py",
    ],
    data = glob(["*.json"]) + [
        "autoencoder_utils.py",
        "//packages/ml:libml_module.so",
        "//packages/object_pose_estimation/apps/autoencoder:simulation_subgraph",
        "//packages/object_pose_estimation/apps/autoencoder:data_subgraph",
        "//packages/viewers:libviewers_module.so",
    ],
    deps = [
        "//engine/pyalice",
        "//packages/ml:pyml",
        "@object_pose_estimation_aae//:py_aae",
    ],
)

isaac_app(
    name = "codebook_generation",
    data = [
        ":simulation_subgraph",
        "@dolly_pose_estimation_model",
    ],
    modules = [
        "behavior_tree",
        "json",
        "ml",
        "object_pose_estimation",
        "viewers",
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
    name = "data_subgraph",
    data = [
        ":simulation_subgraph",
    ],
    modules = [
        "ml",
    ],
    subgraph = "training_data_provider.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_pkg(
    name = "autoencoder_training-pkg",
    srcs = [
        ":autoencoder_training",
    ],
    data = [
        "//packages:py_init",
        "//packages/object_pose_estimation:py_init",
    ],
)
isaac_subgraph(
    name = "pose_estimation_subgraph",
    data = [
        "@dolly_pose_estimation_model",
    ],
    modules = [
        "ml",
        "perception",
        "object_pose_estimation",
    ],
    subgraph = "pose_estimation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "pose_estimation_inference_camerafeed",
    app_json_file = "pose_estimation_inference_camerafeed.app.json",
    data = [
        "pose_estimation_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "@dolly_pose_estimation_model",
        "@resnet_object_detection_model",
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "realsense",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "pose_estimation_inference_imagefeeder",
    app_json_file = "pose_estimation_inference_imagefeeder.app.json",
    data = [
        "pose_estimation_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "//packages/record_replay/apps:replay_subgraph",
        "@dolly_detection_pose_estimation_data",
        "@dolly_pose_estimation_model",
        "@resnet_object_detection_model",
    ],
    modules = [
        "message_generators",
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "pose_estimation_inference_replay",
    app_json_file = "pose_estimation_inference_replay.app.json",
    data = [
        "pose_estimation_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "//packages/record_replay/apps:replay_subgraph",
        "@dolly_detection_pose_estimation_data",
        "@dolly_pose_estimation_model",
        "@resnet_object_detection_model",
    ],
    modules = [
        "message_generators",
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "pose_estimation_inference_sim",
    app_json_file = "pose_estimation_inference_sim.app.json",
    data = [
        "pose_estimation_subgraph",
        "simulation_subgraph",
        "@dolly_pose_estimation_model",
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)