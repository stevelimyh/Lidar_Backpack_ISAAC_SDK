"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "image_superpixels",
    data = [
        "superpixels_subgraph",
        "//packages/rgbd_processing/assets",
    ],
    modules = [
        "message_generators",
        "viewers",
    ],
)

isaac_app(
    name = "live_superpixels",
    data = [
        "superpixels_subgraph",
    ],
    modules = [
        "realsense",
    ],
)

isaac_subgraph(
    name = "superpixels_subgraph",
    modules = [
        "superpixels",
        "rgbd_processing",
        "viewers",
    ],
    subgraph = "superpixels.subgraph.json",
    visibility = ["//visibility:public"],
)
