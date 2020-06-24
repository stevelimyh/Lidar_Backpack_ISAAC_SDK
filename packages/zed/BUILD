"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_component(
    name = "zed_camera",
    visibility = ["//visibility:public"],
    linkopts = [
        "-ldl",
        "-lgomp",
        "-lrt",
    ],
    deps = [
        "//engine/gems/geometry:pinhole",
        "//engine/gems/system:cuda_context",
        "//packages/zed/gems:time_offset_calculator",
        "//third_party:cuda",
        "//third_party:nppc",
        "//third_party:nppial",
        "//third_party:nppidei",
        "//third_party:nppif",
        "//third_party:nppist",
        "//third_party:nppisu",
        "//third_party:zed",
    ],
)

isaac_cc_module(
    name = "zed",
    deps = [
        ":zed_camera",
        "//packages/zed/components:zed_imu_reader"
    ],
)
