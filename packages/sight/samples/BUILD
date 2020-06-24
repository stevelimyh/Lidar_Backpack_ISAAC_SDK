"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_binary")

isaac_cc_binary(
    name = "sight_image_test",
    srcs = ["sight_image_test.cpp"],
    data = [
        "//engine/gems/image/data",
        "@isaac_assets",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/alice/tools:parse_command_line",
        "//engine/alice/tools:websight",
        "//engine/core",
        "//engine/gems/image",
        "//messages",
    ],
)
