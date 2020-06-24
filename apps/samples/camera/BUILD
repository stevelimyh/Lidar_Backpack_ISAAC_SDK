"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_pkg")

py_binary(
    name = "analyse_recording",
    srcs = ["analyse_recording.py"],
    data = [
        "__init__.py",
        "//apps/samples:py_init",
        "//apps:py_init",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
    ],
)

py_binary(
    name = "record_dummy",
    srcs = ["record_dummy.py"],
    data = [
        "__init__.py",
        "//apps/samples:py_init",
        "//apps:py_init",
        "//packages/message_generators:libmessage_generators_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
    ],
)

py_binary(
    name = "record_realsense",
    srcs = ["record_realsense.py"],
    data = [
        "__init__.py",
        "//apps/samples:py_init",
        "//apps:py_init",
        "//packages/realsense:librealsense_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_pkg(
    name = "record-pkg",
    srcs = [
        "analyse_recording",
        "record_dummy",
        "record_realsense",
    ]
)
