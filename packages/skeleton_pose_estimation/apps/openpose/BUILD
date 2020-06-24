"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app")

filegroup(
    name = "validation_dataset",
    srcs = glob([
        "validation_dataset/**/*",
    ]),
)

isaac_app(
    name = "openpose_inference",
    data = [
        ":validation_dataset",
        "@openpose_model",
    ],
    modules = [
        "//packages/message_generators",
        "//packages/ml",
        "//packages/ml:tensorrt",
        "//packages/perception",
        "//packages/skeleton_pose_estimation",
        "//packages/viewers",
    ],
)

isaac_app(
    name = "trt_pose_inference",
    data = [
        ":validation_dataset",
        "@openpose_trt_pose_model",
    ],
    modules = [
        "//packages/message_generators",
        "//packages/ml",
        "//packages/ml:tensorrt",
        "//packages/perception",
        "//packages/skeleton_pose_estimation",
        "//packages/viewers",
    ],
)
