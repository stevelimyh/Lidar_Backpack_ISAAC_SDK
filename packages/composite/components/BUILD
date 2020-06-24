"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_component")

isaac_component(
    name = "composite_metric",
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/math",
        "//packages/composite/gems:parser",
    ],
)

isaac_component(
    name = "composite_atlas",
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/math",
        "//packages/composite/gems:parser",
    ],
)

isaac_component(
    name = "composite_publisher",
    visibility = ["//visibility:public"],
    deps = [
        ":composite_atlas",
        "//messages:tensor",
        "//packages/composite/gems:parser",
    ],
)

isaac_component(
    name = "follow_path",
    visibility = ["//visibility:public"],
    deps = [
        ":composite_metric",
        "//engine/core/math",
        "//messages:tensor",
        "//packages/composite/gems:parser",
        "//packages/composite/gems:serializer",
    ],
)
