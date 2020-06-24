"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
cc_binary(
    name = "libviewers_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/viewers:libviewers_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/viewers:libviewers_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/viewers:libviewers_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/viewers:libviewers_module.so_release"],
    }),
    deps = [
        "//third_party:cudart",
        "//third_party:nppc",
        "//third_party:nppial",
        "//third_party:nppidei",
        "//third_party:nppif",
        "//third_party:nppig",
        "//third_party:nppitc",
        "//third_party:npps",
    ],
    visibility = ["//visibility:public"],
)
