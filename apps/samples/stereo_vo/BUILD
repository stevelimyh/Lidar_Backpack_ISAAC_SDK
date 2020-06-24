"""
Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app")

isaac_app(
    name = "stereo_vo",
    app_json_file = "stereo_vo.app.json",
    modules = [
        "perception:stereo_visual_odometry",
        "utils",
        "viewers",
        "zed",
    ],
)

isaac_py_app(
    name = "svo_zed",
    srcs = [
        "svo_zed.py",
    ],
    args = ["--no-imu"],
    data = [
        "svo_zed.config.json",
        "//packages:py_init",
    ],
    modules = [
        "perception:stereo_visual_odometry",
        "utils",
        "viewers",
        "zed",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_py_app(
    name = "svo_zed_imu",
    srcs = [
        "svo_zed.py",
    ],
    args = ["--imu"],
    data = [
        "svo_zed.config.json",
        "//packages:py_init",
    ],
    main = "svo_zed.py",
    modules = [
        "perception:stereo_visual_odometry",
        "utils",
        "viewers",
        "zed",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_py_app(
    name = "svo_realsense",
    srcs = [
        "svo_realsense.py",
    ],
    data = [
        "svo_realsense.config.json",
        "//packages:py_init",
    ],
    modules = [
        "perception:stereo_visual_odometry",
        "realsense",
        "utils",
        "viewers",
    ],
    deps = [
        "//engine/pyalice",
    ],
)
