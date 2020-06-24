/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/tensor.capnp.h"

namespace isaac {

// This component is specially designed to test `TorchInference` component and is specific to
// torch_inference app. This class sends input to the TorchInference component which loads given
// model and does inference with this input.
class TorchInferenceTestSendInput : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Sends tensors as input to Torch inference
  ISAAC_PROTO_TX(TensorProto, test_input);

  ISAAC_PARAM(float, input_value);
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::TorchInferenceTestSendInput);
