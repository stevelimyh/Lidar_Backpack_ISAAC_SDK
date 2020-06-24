"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component", "isaac_doc_dep")

isaac_cc_module(
    name = "skeleton_pose_estimation",
    deps = [
        ":open_pose_decoder",
    ],
)

isaac_component(
    name = "open_pose_decoder",
    deps = [
        "//engine/core/image",
        "//engine/core/tensor",
        "//engine/gems/image",
        "//engine/gems/tensor",
        "//packages/skeleton_pose_estimation/gems:open_pose_decoder_impl",
    ],
)

isaac_doc_dep(
    name = "doc",
    srcs = [
        "doc/2Dskeleton_pose_estimation.rst",
        "doc/images/01-input.jpg",
        "doc/images/02-heatmap.jpg",
        "doc/images/03-maxpool.jpg",
        "doc/images/04-nonmax.jpg",
        "doc/images/05-equal-confidence.jpg",
        "doc/images/06-equal-thresholded.jpg",
        "doc/images/07-candidates.jpg",
        "doc/images/08-parts-affinity-fields.jpg",
        "doc/images/09-parts-affinity-fields-zoom.jpg",
        "doc/images/10-line-integral.jpg",
        "doc/images/11-edges.jpg",
        "doc/images/12-result.jpg",
        "doc/images/2Dskeleton_pose_estimation_algorithm.jpg",
        "doc/images/inference.jpg",
        "doc/images/inference1.jpg",
        "doc/images/inference2.jpg",
    ],
    visibility = ["//visibility:public"],
    deps = [],
)
