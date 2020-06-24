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
    name = "torch_inference_test_send_input",
    srcs = ["TorchInferenceTestSendInput.cpp"],
    hdrs = ["TorchInferenceTestSendInput.hpp"],
)

isaac_cc_module(
    name = "torch_inference_test_display_output",
    srcs = ["TorchInferenceTestDisplayOutput.cpp"],
    hdrs = ["TorchInferenceTestDisplayOutput.hpp"],
)

isaac_app(
    name = "torch_inference",
    app_json_file = "torch_inference.app.json",
    data = ["@torch_inference_test_assets"],
    modules = [
        "//packages/ml/apps/torch_inference:torch_inference_test_send_input",
        "//packages/ml/apps/torch_inference:torch_inference_test_display_output",
        "ml:torch",
    ],
)
