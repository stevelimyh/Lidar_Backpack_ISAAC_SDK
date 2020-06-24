"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app")

# A trivial GStreamer video test source pipeline example.
isaac_app(
    name = "gstreamer_pipeline",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)

# Demonstrate acquiring, decoding, processing, encoding, and publishing of a video test source
# across distributed Isaac applications. This application is sending the video.
# Run along with "gstreamer_pipeline_distributed_b" application.
isaac_app(
    name = "gstreamer_pipeline_distributed_a",
    modules = [
        "deepstream",
        "perception",
        "sight",
        "viewers",
    ],
)

# Demonstrate acquiring, decoding, processing, encoding, and publishing of a video test source
# across distributed Isaac applications. This application is receiving the video.
# Run along with "gstreamer_pipeline_distributed_a" application.
isaac_app(
    name = "gstreamer_pipeline_distributed_b",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)

# An example of external audio and video pipelines in a single component. No application source
# or sink. The pipelines are part of the robotic application and the service is managed by Isaac
# but there is no direct consumption or production of messages within Isaac: no message ledger
# needed. Useful for extending robotic media support use cases.
isaac_app(
    name = "gstreamer_pipeline_external",
    modules = [
        "deepstream",
    ],
)

# An example of multiple video pipelines in a single component.
isaac_app(
    name = "gstreamer_pipeline_multi",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)

# A USB V4L2 H264 not hardware accelerated 1080p camera: Logitech C920 Pro.
isaac_app(
    name = "logitech_c920pro_cpu",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)

# A USB V4L2 raw YUYV camera: Philips SPC1330NC.
isaac_app(
    name = "philips_spc1330nc",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)

# A 360-degree hemispheric IP RTSP H264 high-resolution camera: SONY SNC-HMX70.
isaac_app(
    name = "sony_snchmx70",
    modules = [
        "deepstream",
        "sight",
        "viewers",
    ],
)
