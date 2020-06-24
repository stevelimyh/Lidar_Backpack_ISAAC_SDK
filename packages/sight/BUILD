"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component", "isaac_doc_dep")

filegroup(
    name = "webroot",
    data = glob(["webroot/**/*"]),
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "sight",
    deps = [
        ":alice_sight",
        ":websight_server",
    ],
)

isaac_component(
    name = "sight_widget",
    deps = [
        "//engine/gems/serialization:json"
    ]
)

isaac_component(
    name = "websight_server",
    data = [
        ":webroot",
        "@isaac_assets",
    ],
    deps = [
        "sight_widget",
        "//engine/gems/algorithm:string_utils",
        "//engine/gems/image",
        "//engine/gems/serialization:files",
        "//engine/gems/serialization:json",
        "//engine/gems/sight",
        "@uwebsockets",
    ],
)

isaac_component(
    name = "alice_sight",
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/sight",
    ],
)

isaac_doc_dep(
    name = "doc",
    srcs = [
        "index.rst",
        "record_replay.rst",
        ] + glob(["doc/*"]),
    visibility = ["//visibility:public"],
)
