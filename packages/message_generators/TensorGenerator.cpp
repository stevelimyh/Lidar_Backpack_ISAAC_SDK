/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "TensorGenerator.hpp"

#include <utility>
#include "engine/gems/tensor/utils.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

namespace {

// Joints (x,y,z) into one value based on a counter.
float Permutate(float x, float y, float z, int i) {
  return (i % 2 ? x : 1.0f - x) + ((i/2) % 2 ? y : 1.0f - y) + ((i/4) % 2 ? z : 1.0f - z);
}

}  // namespace

void TensorGenerator::start() {
  tickPeriodically();

  const Vector3i dimensions = get_dimensions();
  if ((dimensions.array() <= 0).any()) {
    reportFailure("Dimensions must be positive (%d, %d, %d)", dimensions[0], dimensions[1],
                  dimensions[2]);
    return;
  }

  buffer_float32_.resize(dimensions[0], dimensions[1], dimensions[2]);
  for (int i0 = 0; i0 < dimensions[0]; i0++) {
    const float p0 = static_cast<float>(i0) / static_cast<float>(dimensions[0]);
    for (int i1 = 0; i1 < dimensions[1]; i1++) {
      const float p1 = static_cast<float>(i1) / static_cast<float>(dimensions[1]);
      for (int i2 = 0; i2 < dimensions[2]; i2++) {
        const float p2 = static_cast<float>(i2) / static_cast<float>(dimensions[2]);
        buffer_float32_(i0, i1, i2) = Permutate(p0, p1, p2, i2) / 3.0f;
      }
    }
  }

  buffer_int32_.resize(dimensions[0], dimensions[1], dimensions[2]);
  for (int i0 = 0; i0 < dimensions[0]; i0++) {
    for (int i1 = 0; i1 < dimensions[1]; i1++) {
      for (int i2 = 0; i2 < dimensions[2]; i2++) {
        buffer_int32_(i0, i1, i2) = i0 + i1 + i2;
      }
    }
  }
}

void TensorGenerator::tick() {
  switch (get_element_type()) {
    case TensorGeneratorElementType::kFloat32: {
      Tensor3f out(buffer_float32_.dimensions());
      Copy(buffer_float32_, out);
      ToProto(std::move(out), tx_sample().initProto(), tx_sample().buffers());
    } break;
    case TensorGeneratorElementType::kInt32: {
      Tensor3i out(buffer_int32_.dimensions());
      Copy(buffer_int32_, out);
      ToProto(std::move(out), tx_sample().initProto(), tx_sample().buffers());
    } break;
    default:
      reportFailure("Unsupported element type %d", get_element_type());
      return;
  }
  tx_sample().publish();
}

}  // namespace message_generators
}  // namespace isaac
