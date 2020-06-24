"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app")

isaac_app(
    name = "april_tags",
    modules = [
        "//packages/perception:april_tags",
        "sensors:v4l2_camera",
        "viewers",
    ],
)

isaac_py_app(
    name = "april_tags_python",
    srcs = [
        "april_tags_python.py",
    ],
    modules = [
        "//packages/perception:april_tags",
        "realsense",
        "sensors:v4l2_camera",
        "viewers",
        "zed",
    ],
    data = [
        "april_tags_python.config.json",
        "//packages:py_init",
    ],
    deps = [
        "//engine/pyalice",
    ],
)
