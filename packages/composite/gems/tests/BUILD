"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_test_group")

cc_test(
    name = "composite_writer",
    size = "small",
    srcs = ["composite_writer.cpp"],
    deps = [
        "//engine/gems/composite",
        "//engine/gems/math:test_utils",
        "//packages/composite/gems:parser",
        "//packages/composite/gems:serializer",
    ],
)
