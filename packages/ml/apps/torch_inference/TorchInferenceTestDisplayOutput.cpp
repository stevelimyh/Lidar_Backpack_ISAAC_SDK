/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "TorchInferenceTestDisplayOutput.hpp"

#include <cmath>

#include "messages/tensor.hpp"

namespace isaac {

void TorchInferenceTestDisplayOutput::start() {
  tickOnMessage(rx_test_output());
}

void TorchInferenceTestDisplayOutput::tick() {
  TensorConstView2f buffer_tensor;
  if (!FromProto(rx_test_output().getProto(), rx_test_output().buffers(), buffer_tensor)) {
    return;
  }
  LOG_INFO("Torch Inference test output: %f", buffer_tensor(0, 0));
}

}  // namespace isaac
