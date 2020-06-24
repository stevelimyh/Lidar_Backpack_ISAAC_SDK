
"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

filegroup(
    name = "libmap_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/map:libmap_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/map:libmap_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "public_headers",
    hdrs = [
        "Layer.hpp",
        "WaypointMapLayer.hpp",
    ],
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/map:libmap_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/map:libmap_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "kinematic",
    deps = [
        "kinematic_tree",
    ],
)

isaac_component(
    name = "kinematic_tree",
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/math:dual_quaternion",
        "//engine/gems/kinematic_tree",
        "//engine/gems/kinematic_tree:json_loader",
        "//packages/composite/gems:parser",
    ],
)
