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
    name = "direction_of_audio_event",
    srcs = ["DirectionOfAudioEvent.cpp"],
    hdrs = ["DirectionOfAudioEvent.hpp"],
    deps = [
        "//engine/gems/sight",
        "//engine/gems/state:io",
        "//messages/state:audio",
    ],
)

isaac_app(
    name = "sound_source_localization",
    modules = [
        "//apps/samples/sound_source_localization:direction_of_audio_event",
        "audio",
    ],
)
