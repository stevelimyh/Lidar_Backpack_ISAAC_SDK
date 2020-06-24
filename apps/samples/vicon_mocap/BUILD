"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "vicon_mocap_test",
    srcs = [
        "MocapTest.cpp",
    ],
    hdrs = [
        "MocapTest.hpp",
    ],
    tags = ["manual"],
    deps = [
        "//engine/alice/tools:parse_command_line",
        "//external:gflags",
    ],
)

isaac_app(
    name = "vicon_mocap",
    app_json_file = "vicon_mocap.app.json",
    modules = [
        "//apps/samples/vicon_mocap:vicon_mocap_test",
        "sensors:vicon",
    ],
    tags = ["manual"],
)
