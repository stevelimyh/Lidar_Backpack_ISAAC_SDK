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
// torch_inference app. This class displays the output received from the TorchInference component.
class TorchInferenceTestDisplayOutput : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Receives tensor output from the Torch inference
  ISAAC_PROTO_RX(TensorProto, test_output);
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::TorchInferenceTestDisplayOutput);
