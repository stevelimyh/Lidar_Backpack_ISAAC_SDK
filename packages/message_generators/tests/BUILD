"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "flatscan_consumption",
    srcs = ["flatscan_consumption.cpp"],
    deps = [
        "//engine/alice",
        "//packages/message_generators:flatscan_generator",
        "@gtest//:main",
    ],
)

cc_test(
    name = "image_loader",
    size = "small",
    tags = ["performance"],
    srcs = ["image_loader.cpp"],
    deps = [
        "//engine/alice",
        "//engine/gems/image",
        "//packages/message_generators:image_loader",
        "@gtest//:main",
    ],
)
