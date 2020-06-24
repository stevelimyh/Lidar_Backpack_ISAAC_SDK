"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_pkg", "isaac_subgraph")

isaac_subgraph(
    name = "freespace_dnn_data_annotation_subgraph",
    data = [
        "//packages/record_replay/apps:replay_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
    ],
    modules = [
        "superpixels",
    ],
    subgraph = "freespace_dnn_data_annotation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "freespace_dnn_inference_subgraph",
    data = [
        "freespace_dnn_inference_config",
    ],
    modules = [
        "behavior_tree",
        "ml",
        "ml:tensorflow",
        "perception",
        "sight",
        "viewers",
    ],
    subgraph = "freespace_dnn_inference.subgraph.json",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "freespace_dnn_inference_config",
    data = [
        ":freespace_dnn_inference_building_k_tensorflow.config.json",
        ":freespace_dnn_inference_indoor_tensorflow.config.json",
        ":freespace_dnn_inference_medium_warehouse_tensorflow.config.json",
        ":freespace_dnn_inference_medium_warehouse_tensorrt.config.json",
        ":freespace_dnn_inference_sidewalk_tensorflow.config.json",
        ":freespace_dnn_inference_sidewalk_tensorrt.config.json",
        "@path_segmentation_pretrained_models",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "freespace_dnn_inference_image",
    data = [
        "freespace_dnn_inference_subgraph",
        "@path_segmentation_images",
    ],
    modules = [
        "message_generators",
    ],
)

isaac_app(
    name = "freespace_dnn_inference_replay",
    data = [
        "freespace_dnn_inference_subgraph",
        "//packages/record_replay/apps:replay_subgraph",
        "@path_segmentation_logs",
    ],
)

isaac_app(
    name = "freespace_dnn_inference_unity3d",
    data = [
        "freespace_dnn_inference_subgraph",
        ":freespace_dnn_inference_unity3d_building_k.config.json",
        ":freespace_dnn_inference_unity3d_medium_warehouse.config.json",
        ":freespace_dnn_inference_unity3d_sidewalk.config.json",
        "//packages/navsim/apps:navsim_training_subgraph",
    ],
    modules = [
        "map",
    ],
)

isaac_app(
    name = "freespace_dnn_inference_v4l2",
    data = [
        "freespace_dnn_inference_subgraph",
    ],
    modules = [
        "sensors:v4l2_camera",
    ],
)

isaac_subgraph(
    name = "freespace_dnn_training_subgraph",
    modules = [
        "map",
        "ml",
        "sight",
        "viewers",
    ],
    subgraph = "freespace_dnn_training.subgraph.json",
    visibility = ["//visibility:public"],
)

py_binary(
    name = "freespace_dnn_training",
    srcs = [
        "freespace_dnn_training.py",
    ],
    data = [
        "__init__.py",
        "freespace_dnn_training.config.json",
        "freespace_dnn_training_freeze_model.py",
        "freespace_dnn_training_models.py",
        "freespace_dnn_training_unity3d.app.json",
        "freespace_dnn_training_unity3d_building_k.config.json",
        "freespace_dnn_training_unity3d_medium_warehouse.config.json",
        "freespace_dnn_training_unity3d_rng_warehouse.config.json",
        "freespace_dnn_training_unity3d_sidewalk.config.json",
        ":freespace_dnn_training_subgraph",
        "//packages:py_init",
        "//packages/freespace_dnn:py_init",
        "//packages/map:libmap_module.so",
        "//packages/ml:libml_module.so",
        "//packages/navsim/apps:navsim_training_subgraph",
        "//packages/viewers:libviewers_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
        "//packages/ml:pyml",
    ],
)

isaac_pkg(
    name = "freespace_dnn_training-pkg",
    srcs = [":freespace_dnn_training"],
    visibility = ["//visibility:public"],
)
