"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library", "isaac_cc_module", "isaac_component")

isaac_cc_library(
    name = "birth",
    srcs = [],
    hdrs = [
        "Birth.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
    ],
)

isaac_cc_library(
    name = "death",
    srcs = [],
    hdrs = [
        "Death.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/core/tensor",
    ],
)

isaac_cc_library(
    name = "reward",
    srcs = [],
    hdrs = [
        "Reward.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/core/tensor",
    ],
)

isaac_cc_library(
    name = "state_refiner",
    srcs = [],
    hdrs = [
        "StateRefiner.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/core/tensor",
    ],
)
