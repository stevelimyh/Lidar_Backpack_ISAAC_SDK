"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_doc_dep")

isaac_doc_dep(
    name = "doc",
    srcs = glob([
        "doc/*.rst",
        "doc/images/*.png",
    ]),
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "livox",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/livox/components:livox_lidar",
    ],
)
