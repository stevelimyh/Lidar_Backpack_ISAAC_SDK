"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app")

# Application to generate maps and record logs of a building with Carter robot. The application
# uses the GMapping tool. The joystick is used to move the robot about the building. We recommend
# to record the run. After recording, use the log_gmapping application to generate a map with
# different mapping parameters.
isaac_app(
    name = "gmapping",
    data = [
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots",
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
    ],
)
