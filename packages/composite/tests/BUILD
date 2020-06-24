"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "composite_metric",
    size = "small",
    srcs = ["composite_metric.cpp"],
    deps = [
        "//engine/alice",
        "//packages/composite/components:composite_metric",
        "@gtest//:main",
    ],
)

py_test(
    name = "composite_publisher",
    srcs = ["composite_publisher.py"],
    data = [
        "composite_atlas_generator_gen",
        "//packages/composite:libcomposite_module.so",
    ],
    tags = ["pytest"],
    deps = [
        "//engine/pyalice",
    ],
)

py_test(
    name = "follow_path",
    srcs = ["follow_path.py"],
    data = [
        "composite_atlas_generator_gen",
        "//packages/composite:libcomposite_module.so",
    ],
    tags = ["pytest"],
    deps = [
        "//engine/pyalice",
    ],
)

py_binary(
    name = "composite_atlas_generator",
    srcs = ["composite_atlas_generator.py"],
    data = ["//messages:capnp"],
    deps = ["//engine/pyalice"],
)

genrule(
    name = "composite_atlas_generator_gen",
    srcs = ["//messages:capnp"],
    outs = ["waypoints/kv"],
    cmd = """
      $(location composite_atlas_generator)
      cp waypoints/kv $@
      """,
    tools = ["composite_atlas_generator"],
)
