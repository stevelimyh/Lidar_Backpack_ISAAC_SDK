/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "serializer.hpp"

#include <algorithm>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "capnp/serialize.h"
#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace composite {

void Serializer::setSchema(Schema schema) {
  requested_schema_ = std::move(schema);
}

bool Serializer::serialize(const VectorXd& data, ::CompositeProto::Builder builder,
                           std::vector<SharedBuffer>& buffers) {
  // Fails on invalid schema
  if (requested_schema_ == std::nullopt) {
    return false;
  }
  const int data_size = data.size();
  // Fails on mis-matching scheme size and data size
  if (data_size != requested_schema_->getElementCount()) {
    return false;
  }

  // Writes schema into proto
  WriteSchema(*requested_schema_, builder);

  // Writes values
  Tensor1d tensor(data.size());
  const double* data_ptr = data.data();
  std::copy(data_ptr, data_ptr + data_size, tensor.element_wise_begin());
  ToProto(std::move(tensor), builder.initValues(), buffers);
  return true;
}

bool Serializer::serialize(Tensor3d states, ::CompositeProto::Builder builder,
                           std::vector<SharedBuffer>& buffers) {
  // Fails on invalid schema
  if (requested_schema_ == std::nullopt) {
    return false;
  }
  const int state_dimension = states.dimensions()[2];
  // Fails on mis-matching scheme size and data size
  if (state_dimension != requested_schema_->getElementCount()) {
    return false;
  }

  // Writes schema into proto
  WriteSchema(*requested_schema_, builder);

  // Writes values
  ToProto(std::move(states), builder.initValues(), buffers);
  return true;
}

}  // namespace composite
}  // namespace isaac
