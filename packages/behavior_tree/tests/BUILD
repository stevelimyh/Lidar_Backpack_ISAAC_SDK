"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library", "isaac_cc_test_group")

isaac_cc_library(
    name = "utils",
    srcs = ["utils.cpp"],
    hdrs = ["utils.hpp"],
    deps = [
        "//engine/alice",
        "//packages/behavior_tree/components",
    ],
)

isaac_cc_test_group(
    srcs = ["repeat.cpp"],
    tags = ["host"],
    deps = [
        "utils",
        "//engine/alice",
        "//packages/behavior_tree/components",
    ],
)

isaac_cc_test_group(
    srcs = ["combined.cpp"],
    tags = ["host"],
    deps = [
        "utils",
        "//engine/alice",
        "//packages/behavior_tree/components",
    ],
)

isaac_cc_test_group(
    srcs = ["small.cpp"],
    tags = ["performance"],
    deps = [
        "utils",
        "//engine/alice",
        "//packages/behavior_tree/components",
    ],
)
