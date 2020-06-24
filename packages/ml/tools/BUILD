"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_pkg")

py_binary(
    name = "freeze_tensorflow_model_tool",
    srcs = [
        "freeze_tensorflow_model.py",
    ],
    main = "freeze_tensorflow_model.py",
    visibility = ["//visibility:public"],
)

isaac_pkg(
    name = "freeze_tensorflow_model_tool-pkg",
    srcs = [":freeze_tensorflow_model_tool"],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "tensorflow_to_tensorrt_tool",
    srcs = [
        "tensorflow_to_tensorrt.py",
    ],
    deps = [
        "@tensorrt_x86_64//:uff_library",
    ],
    main = "tensorflow_to_tensorrt.py",
    visibility = ["//visibility:public"],
)

isaac_pkg(
    name = "tensorflow_to_tensorrt_tool-pkg",
    srcs = [":tensorflow_to_tensorrt_tool"],
    visibility = ["//visibility:public"],
)
