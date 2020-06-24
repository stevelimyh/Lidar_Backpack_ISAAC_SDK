/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "messages/tensor.capnp.h"

namespace isaac {
namespace message_generators {

// TensorAssignmentGenerator creates lists of assignment tensors based on the input dimensions
// It provides option to select between random assignment where every value in a row can be either
// 0 or 1 and uniform assignment where every value is 1.
class BinaryTensorGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Produces tensor of type int with entries set randomly to 0 or 1.
  ISAAC_PROTO_TX(TensorProto, assignment);

  // Dimensions of the generated rank 2 tensor
  ISAAC_PARAM(Vector2i, dimensions, Vector2i(1, 100));
  // Probability for assignment
  ISAAC_PARAM(double, probability, 0.5);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::BinaryTensorGenerator);
