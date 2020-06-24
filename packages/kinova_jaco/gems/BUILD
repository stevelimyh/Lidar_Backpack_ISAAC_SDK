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
    name = "kinova_jaco",
    srcs = ["kinova_jaco_api.cpp"],
    hdrs = ["kinova_jaco_api.hpp"],
    linkopts = ["-ldl"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/serialization:json",
        "@kinova_jaco",
    ],
)
