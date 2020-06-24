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
#include "engine/core/tensor/tensor.hpp"
#include "messages/detections.capnp.h"

namespace isaac {
namespace message_generators {

// Binarizes the segmentation output based on a user-defined threshold
class ConfusionMatrixGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output segmentation prediction with regulated probabilities
  ISAAC_PROTO_TX(ConfusionMatrixProto, confusion_matrix);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::ConfusionMatrixGenerator);
