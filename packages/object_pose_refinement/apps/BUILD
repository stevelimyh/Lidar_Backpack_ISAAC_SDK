"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

filegroup(
    name = "config",
    srcs = [
        "pose_refinement_dolly.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "pose_refinement_subgraph",
    data = [
        "@industrial_dolly_pose_refinement_data",
    ],
    modules = [
        "ml",
        "object_pose_refinement",
        "surflets"
    ],
    subgraph = "pose_refinement.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "pose_refinement_replay",
    app_json_file = "pose_refinement_replay.app.json",
    data = [
        ":config",
        "pose_refinement_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:config",
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@industrial_dolly_pose_estimation_data",
        "@industrial_dolly_pose_refinement_data",
    ],
    modules = [
        "object_pose_refinement",
        "sight",
        "viewers",
    ],
    visibility = ["//visibility:public"]
)

isaac_app(
    name = "pose_refinement_camerafeed",
    app_json_file = "pose_refinement_camerafeed.app.json",
    data = [
        ":config",
        "pose_refinement_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:config",
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@industrial_dolly_pose_refinement_data",
    ],
    modules = [
        "object_pose_refinement",
        "realsense",
        "viewers",
    ],
)

isaac_app(
    name = "pose_refinement_imagefeed",
    app_json_file = "pose_refinement_imagefeed.app.json",
    data = [
        ":config",
        "pose_refinement_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:config",
        "//packages/object_pose_refinement/apps/assets",
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@industrial_dolly_pose_refinement_data",
    ],
    modules = [
        "message_generators",
        "object_pose_refinement",
        "viewers",
    ],
)
