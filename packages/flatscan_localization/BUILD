
"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

filegroup(
    name = "libflatscan_localization_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/flatscan_localization:libflatscan_localization_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/flatscan_localization:libflatscan_localization_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

