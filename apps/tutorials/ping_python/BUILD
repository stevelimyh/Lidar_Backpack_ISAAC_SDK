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
    name = "ping_python",
    srcs = [
        "__init__.py",
        "ping_python.py",
    ],
    data = [
        "ping_python.app.json",
        "//apps:py_init",
        "//apps/tutorials:py_init",
    ],
    deps = [
        "//engine/pyalice",
    ],
)

isaac_pkg(
    name = "ping_python-pkg",
    srcs = ["ping_python"],
)
