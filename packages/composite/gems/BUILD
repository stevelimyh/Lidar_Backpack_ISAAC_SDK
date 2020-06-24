"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "parser",
    srcs = [
        "fragment_index.cpp",
        "measure.cpp",
        "parser.cpp",
        "schema.cpp",
    ],
    hdrs = [
        "fragment_index.hpp",
        "measure.hpp",
        "parser.hpp",
        "quantity.hpp",
        "schema.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/array",
        "//engine/core/buffers",
        "//engine/core/tensor",
        "//engine/core/tensor:element_type",
        "//engine/gems/algorithm:timeseries",
        "//engine/gems/composite",
        "//engine/gems/serialization",
        "//messages:composite_proto",
        "//messages:math",
        "//messages:tensor",
    ],
)

isaac_cc_library(
    name = "serializer",
    srcs = [
        "serializer.cpp",
    ],
    hdrs = [
        "serializer.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parser",
        "//engine/core",
        "//engine/core/array",
        "//engine/core/buffers",
        "//engine/core/tensor",
        "//engine/core/tensor:element_type",
        "//engine/gems/composite",
        "//engine/gems/serialization",
        "//messages:composite_proto",
        "//messages:math",
        "//messages:tensor",
    ],
)
