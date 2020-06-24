"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libv4l2_camera_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libv4l2_camera_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libv4l2_camera_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libv4l2_camera_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libv4l2_camera_module.so_release"],
    }),
    deps = [
        "//third_party:cudart",
        "//third_party:nppc",
        "//third_party:nppicc",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libvelodyne_driver_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libvelodyne_driver_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libvelodyne_driver_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libjoystick_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libjoystick_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libjoystick_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libvicon_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libvicon_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libvicon_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libzed_camera_module.so",
    tags = ["manual"],
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libzed_camera_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libzed_camera_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libzed_camera_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libzed_camera_module.so_release"],
    }),
    deps = [
        "//third_party:zed",
        "//third_party:nppist",
        "//third_party:nppisu",
        "//third_party:nppial",
        "//third_party:nppidei",
        "//third_party:nppif",
    ],
    linkshared = True,
    visibility = ["//visibility:public"],
)

filegroup(
    name = "librealsense_camera_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:librealsense_camera_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:librealsense_camera_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libserial_bmi160_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/sensors:libserial_bmi160_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libserial_bmi160_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libargus_camera_module.so",
    tags = ["manual"],
    data = select({
        "//engine/build:platform_x86_64": [],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/sensors:libargus_camera_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)
