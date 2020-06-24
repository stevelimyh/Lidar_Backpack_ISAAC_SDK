"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "time_offset_calculator",
    size = "small",
    tags = ["host"],
    srcs = ["time_offset_calculator.cpp"],
    deps = [
        "//packages/zed/gems:time_offset_calculator",
        "@gtest//:main",
    ],
)
