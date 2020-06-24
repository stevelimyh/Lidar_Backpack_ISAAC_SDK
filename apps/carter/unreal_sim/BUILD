"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "carter_sim_subgraph",
    modules = [
        "flatsim",
        "sight",
    ],
    subgraph = "carter_sim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "2d_carter_sim_subgraph",
    data = [
        ":carter_sim_subgraph",
        "//packages/navigation/apps:scan_flattener_subgraph",
    ],
    subgraph = "2d_carter_sim.subgraph.json",
    visibility = ["//visibility:public"],
)
