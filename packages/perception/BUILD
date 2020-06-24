"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libstereo_visual_odometry_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/perception:libstereo_visual_odometry_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/perception:libstereo_visual_odometry_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/perception:libstereo_visual_odometry_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/perception:libstereo_visual_odometry_module.so_release"],
    }),
    deps = [
        "@elbrus_vo",
        "//third_party:cudart",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libperception_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/perception:libperception_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/perception:libperception_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/perception:libperception_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/perception:libperception_module.so_release"],
    }),
    deps = [
        "//third_party:cudart",
        "//third_party:nppc",
        "//third_party:nppitc",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libapril_tags_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/perception:libapril_tags_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/perception:libapril_tags_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)
