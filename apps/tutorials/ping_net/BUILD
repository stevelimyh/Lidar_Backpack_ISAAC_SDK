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
    name = "ping_component",
    srcs = [
        "ping.cpp",
    ],
    hdrs = [
        "ping.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [],
)

isaac_cc_module(
    name = "pong_component",
    srcs = [
        "pong.cpp",
    ],
    hdrs = [
        "pong.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [],
)

isaac_app(
    name = "ping",
    app_json_file = "ping.app.json",
    modules = [
        "viewers",
        "//apps/tutorials/ping_net:ping_component",
    ],
)

isaac_app(
    name = "pong",
    app_json_file = "pong.app.json",
    modules = [
        "viewers",
        "//apps/tutorials/ping_net:pong_component",
    ],
)
