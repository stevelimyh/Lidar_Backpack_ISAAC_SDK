"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_pkg")

isaac_app(
    name = "navsim_server",
    data = [
        "//apps/navsim/multirobot/scenarios",
        "//engine/alice/c_api:libisaac_c_api.so",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_client",
    data = [
        "//apps/navsim/multirobot/configs:client_config",
        "//apps/navsim/multirobot/scenarios",
        "//engine/alice/c_api:libisaac_c_api.so",
    ],
    visibility = ["//visibility:public"],
)

isaac_pkg(
    name = "navsim-pkg",
    srcs = [
        ":navsim_client",
        ":navsim_server",
        "//packages/navsim/apps:navsim"
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_navigate",
    data = [
        "//apps/assets/maps",
        "//apps/navsim/multirobot/configs:navigate_config",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "map",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "mosaic",
    modules = [
        "ml",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)
