"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

py_test(
    name = "verify_confusion_matrices",
    srcs = ["verify_confusion_matrices.py"],
    data = [
        "//packages/detect_net/apps:config",
        "//packages/detect_net/apps:detect_net_inference",
        "//packages/ml:libevaluation_module.so",
        "//packages/ml:libml_module.so",
        "//packages/navsim/apps:navsim_training_subgraph",
        "//packages/record_replay/apps:replay_subgraph",
        "@dolly_detection_test_data",
        "evaluate_object_detection_log.app.json",
        "evaluate_object_detection_simulation.app.json",
    ],
    deps = [
        "//engine/pyalice"
    ],
    tags = ["manual"]
)
