"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libdisparity_dnn_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/stereo_depth:libdisparity_dnn_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/stereo_depth:libdisparity_dnn_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/stereo_depth:libdisparity_dnn_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/stereo_depth:libdisparity_dnn_module.so_release"],
    }),
    deps = [
        "//third_party:tensorrt",
        "//third_party:cublas",
        "//third_party:cudart",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libutils_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/stereo_depth:libutils_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/stereo_depth:libutils_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libcoarse_to_fine_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/stereo_depth:libcoarse_to_fine_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/stereo_depth:libcoarse_to_fine_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)
