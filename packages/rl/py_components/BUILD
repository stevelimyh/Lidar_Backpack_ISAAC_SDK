"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

filegroup(
    name = "py_init",
    data = ["__init__.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "off_policy_trainer",
    srcs = [
        "off_policy_trainer.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/pyalice",
    ],
)

py_library(
    name = "soft_actor_critic",
    srcs = [
        "soft_actor_critic.py",
        "soft_actor_critic_utils.py",
    ],
    visibility = ["//visibility:public"],
)
