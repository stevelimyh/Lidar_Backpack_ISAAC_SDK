/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/algorithm/timeseries.hpp"
#include "messages/composite.capnp.h"
#include "messages/tensor.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

// Parses composite message for specified entity values.
class Serializer {
 public:
  // Sets the desired schema which needs to be parsed and updates parsing lookup table if necessary
  void setSchema(Schema schema);

  // Serializes data into CompositeProto
  bool serialize(const VectorXd& data, ::CompositeProto::Builder builder,
                 std::vector<SharedBuffer>& buffers);

  // Serializes batches of states into a composite proto. `states` is a tensor with the following
  // layout: (batch, time, state).
  bool serialize(Tensor3d states, ::CompositeProto::Builder builder,
                 std::vector<SharedBuffer>& buffers);

  template <int N>
  bool serialize(const Timeseries<Vector<double, N>, double>& series,
                 const Quantity timestamp_quantity, ::CompositeProto::Builder builder,
                 std::vector<SharedBuffer>& buffers);

 private:
  // The requested schema
  std::optional<Schema> requested_schema_;

  // Offsets to access requested quantities from values vector and size of quantities
  std::vector<std::pair<int, int>> offsets_and_counts_;
};

// Serializes data sequence into CompositeProto
template <int N>
bool Serializer::serialize(const Timeseries<Vector<double, N>, double>& series,
                           const Quantity timestamp_quantity, ::CompositeProto::Builder builder,
                           std::vector<SharedBuffer>& buffers) {
  // Fails on invalid schema
    if (requested_schema_ == std::nullopt) {
    return false;
  }
  // Fails on mis-matching scheme size and provided size
  if (N != requested_schema_->getElementCount()) {
    return false;
  }
  ASSERT(timestamp_quantity.getElementCount() == 1, "Timestamp has to be scalar value");

  // Writes schema into proto
  WriteSchema(*requested_schema_, builder);

  const int series_size = series.size();
  // Succeeds if nothing to write
  if (series_size <= 0) {
    return true;
  }

  // Specified timestamp quantity needs to be overwritten
  const auto timestamp_index = requested_schema_->findQuantityValueIndex(timestamp_quantity);
  if (!timestamp_index) {
    return false;
  }

  // Creates and writes values
  Tensor2d values(Vector2i{series_size, N});
  for (int idx = 0; idx < series_size; ++idx) {
    const auto& entry = series.at(idx);
    const double* const data_ptr = entry.state.data();
    auto slice_view = values.slice(idx);
    // Copies value for the entry
    std::copy(data_ptr, data_ptr + N, slice_view.element_wise_begin());
    // Overrides specified timestamp quantity
    slice_view(*timestamp_index) = entry.stamp;
  }
  ToProto(std::move(values), builder.initValues(), buffers);

  return true;
}

}  // namespace composite
}  // namespace isaac
