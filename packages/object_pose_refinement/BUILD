"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
cc_binary(
    name = "libobject_pose_refinement_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/object_pose_refinement:libobject_pose_refinement_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/object_pose_refinement:libobject_pose_refinement_module.so_release"],
    }),
    data = select({
        "//engine/build:platform_x86_64": ["//packages_x86_64/object_pose_refinement:libobject_pose_refinement_module.so_release"],
        "//engine/build:platform_jetpack43": ["//packages_jetpack43/object_pose_refinement:libobject_pose_refinement_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "py_init",
    data = ["__init__.py"],
    visibility = ["//visibility:public"],
)
