"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "cuda_ping_components",
    visibility = ["//visibility:public"],
    deps = [":cuda_image_ping",
            ":cuda_image_pong"],
)
isaac_component(
    name = "cuda_image_ping",
    visibility = ["//visibility:public"],
    deps = ["//engine/core/image",],
)

isaac_component(
    name = "cuda_image_pong",
    visibility = ["//visibility:public"],
    deps = ["//engine/core/image",],
)

isaac_app(
    name = "cuda_ping_pong",
    modules = [
        "//apps/tutorials/cuda_ping_pong:cuda_ping_components",
    ],
)
