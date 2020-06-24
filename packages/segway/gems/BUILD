"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "segway",
    srcs = [
        "segway.cpp",
    ],
    hdrs = [
        "rmp_types.hpp",
        "segway.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)
