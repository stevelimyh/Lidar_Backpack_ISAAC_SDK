/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/message_generators/BinaryTensorGenerator.hpp"

#include <random>
#include <utility>

#include "engine/alice/node.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

void BinaryTensorGenerator::start() {
  tickPeriodically();
}

void BinaryTensorGenerator::tick() {
  const Vector2i dimensions = get_dimensions();
  if ((dimensions.array() <= 0).any()) {
    reportFailure("Dimensions must be positive (%d, %d)", dimensions[0], dimensions[1]);
    return;
  }
  Tensor2i out;
  out.resize(dimensions[0], dimensions[1]);
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);
  Fill(out, 0);
  for (int i = 0; i < dimensions[0]; i++) {
    for (int j = 0; j < dimensions[1]; j++) {
      if (distribution(generator) <= get_probability()) {
        out(i, j) = 1;
      }
    }
  }
  ToProto(std::move(out), tx_assignment().initProto(), tx_assignment().buffers());
  tx_assignment().publish(node()->clock()->time());
}

}  // namespace message_generators
}  // namespace isaac
