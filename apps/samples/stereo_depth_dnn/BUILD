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
    name = "stereo_depth_dnn",
    app_json_file = "stereo_depth_dnn.app.json",
    data = [
        ":json",
        "@redtail//:resnet18_2D_weights_and_plan",
    ],
    modules = [
        "perception",
        "stereo_depth:disparity_dnn",
        "stereo_depth:utils",
        "viewers",
        "zed",
    ],
    script = "stereo_depth_dnn.sh",
)

filegroup(
    name = "json",
    srcs = select({
        "@com_nvidia_isaac//engine/build:platform_x86_64": ["stereo_depth_dnn.config.x64.json"],
        "@com_nvidia_isaac//engine/build:platform_jetpack43": ["stereo_depth_dnn.config.xavier.json"],
    }),
)
