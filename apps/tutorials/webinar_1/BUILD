"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "go_to_mockup",
    srcs = ["GoToMockup.cpp"],
    hdrs = ["GoToMockup.hpp"],
)

isaac_cc_module(
    name = "goal_generator",
    srcs = ["GoalGenerator.cpp"],
    hdrs = ["GoalGenerator.hpp"],
)

isaac_cc_module(
    name = "my_goal_generator",
    srcs = ["MyGoalGenerator.cpp"],
    hdrs = ["MyGoalGenerator.hpp"],
)

isaac_app(
    name = "goal_generator_test",
    modules = [
        "//apps/tutorials/webinar_1:go_to_mockup",
        "//apps/tutorials/webinar_1:goal_generator",
    ],
)

isaac_app(
    name = "my_goal_generator_test",
    modules = [
        "//apps/tutorials/webinar_1:go_to_mockup",
        "//apps/tutorials/webinar_1:my_goal_generator",
    ],
)

isaac_app(
    name = "goal_generator_sim",
    data = [
        "//apps/assets/maps",
        "//apps/carter/unreal_sim:2d_carter_sim_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "//apps/tutorials/webinar_1:goal_generator",
        "imu",
        "map",
        "navigation",
        "perception",
        "planner",
        "viewers",
    ],
)

isaac_app(
    name = "goal_generator_navsim",
    data = [
        "//apps/assets/maps",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "//apps/tutorials/webinar_1:goal_generator",
        "behavior_tree",
        "map",
        "navigation",
    ],
)

isaac_app(
    name = "goal_generator_real",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "//apps/tutorials/webinar_1:goal_generator",
        "imu",
        "map",
        "navigation",
        "perception",
        "planner",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
        "viewers",
    ],
)
