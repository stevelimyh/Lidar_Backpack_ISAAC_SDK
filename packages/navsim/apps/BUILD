"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "navsim_viewer_tcp",
    data = [
        "navsim_training_subgraph",
    ],
    modules = [
        "ml",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "monitor_subgraph",
    modules = [
        "navigation",
        "navsim",
        "utils",
    ],
    subgraph = "monitor.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "noisy_scan_flattening",
    modules = [
        "flatsim",
        "perception",
    ],
    subgraph = "noisy_scan_flattening.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "navsim_tcp_subgraph",
    modules = [
        "navsim",
    ],
    subgraph = "navsim_tcp.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "navsim_navigation_subgraph",
    data = [
        "//packages/navsim/apps:noisy_scan_flattening",
    ],
    modules = [
        "imu",
        "navsim",
        "utils",
    ],
    subgraph = "navsim_navigation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "navsim_training_subgraph",
    modules = [
        "navsim",
        "ml",
        "utils",
    ],
    subgraph = "navsim_training.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim",
    data = [
        "//engine/alice/c_api:libisaac_c_api.so",
        "//packages/navsim/scenarios",
    ],
    visibility = ["//visibility:public"],
)
