/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/algorithm/timeseries.hpp"
#include "messages/composite.capnp.h"
#include "messages/tensor.hpp"
#include "packages/composite/gems/fragment_index.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

// Parses composite message for specified entity values.
class Parser {
 public:
  // Sets the desired schema which needs to be parsed and updates parsing lookup table if necessary
  void requestSchema(Schema schema);

  // Parses quantities setup with `requestSchema` from a CompositeProto and stores them
  // sequentially in a pre-allocated array. Returns false if the composite proto was incompatible
  // with the requested quantities.
  bool parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
             double* output_begin, double* output_end);

  // Similar to parse but stores quantities in an Eigen vector.
  template <int N>
  bool parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
             Vector<double, N>& vector) {
    return parse(reader, buffers, vector.data(), vector.data() + vector.size());
  }

  // Similar to parse but stores batches of quantities in an Eigen matrix. Each column is one batch.
  // In case timeseries are present the first element in the timeseries is taken.
  template <int N>
  bool parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
             Eigen::Matrix<double, N, Eigen::Dynamic>& batches);

  // Parses a time series
  template <int N>
  bool parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
             const std::string& time_entity, Timeseries<Vector<double, N>, double>& series);

 private:
  // Parses schema from incoming message and updates parsing lookup table if necessary
  void parseSchema(::CompositeProto::Reader reader);

  // Sets the source schema received via proto
  void setReceivedSchema(std::optional<Schema> schema);

  // The requested schema
  Schema requested_schema_;

  // The schema of the last received message
  std::optional<Schema> received_schema_;

  // Helper index for copying elements
  std::optional<FragmentIndex> fragment_index_;
};

template <int N>
bool Parser::parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
                   Eigen::Matrix<double, N, Eigen::Dynamic>& batches) {
  // Parse the incoming schema
  parseSchema(reader);
  // Fail in case of incompatible schema
  if (fragment_index_ == std::nullopt) {
    return false;
  }

  // Get tensor storing quantity values
  TensorConstView3d values;
  if (!FromProto(reader.getValues(), buffers, values)) {
    return false;
  }

  const int src_state_count = values.dimensions()[2];
  if (src_state_count != received_schema_->getElementCount()) {
    return false;
  }

  const int batch_count = values.dimensions()[0];
  const int dst_state_count = requested_schema_.getElementCount();
  if (N != Eigen::Dynamic && N != dst_state_count) {
    return false;
  }
  batches.resize(dst_state_count, batch_count);

  // Copy values for each batch from tensor to column in output matrix
  for (int batch = 0; batch < batch_count; batch++) {
    const auto src = values.matrix(batch, 0);
    fragment_index_->copyFragments(src.data(), src.size(), &batches(0, batch), dst_state_count);
  }

  return true;
}

template <int N>
bool Parser::parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
                   const std::string& time_entity, Timeseries<Vector<double, N>, double>& series) {
  // Parse the incoming schema
  parseSchema(reader);
  // Fail in case of incompatibel schema
  if (fragment_index_ == std::nullopt) {
    return false;
  }

  // Find the time entity
  const auto time_index =
      received_schema_->findQuantityValueIndex(Quantity::Scalar(time_entity, Measure::kTime));
  if (time_index == std::nullopt) {
    // Time index not found in received schema
    return false;
  }

  // Get tensor storing quantity values
  TensorConstView2d values;
  if (!FromProto(reader.getValues(), buffers, values)) {
    return false;
  }
  if (values.dimensions()[1] != received_schema_->getElementCount()) {
    return false;
  }

  const int dst_state_count = requested_schema_.getElementCount();
  if (N != Eigen::Dynamic && N != dst_state_count) {
    return false;
  }

  series.clear();
  for (int row = 0; row < values.dimensions()[0]; row++) {
    const auto slice = values.slice(row);
    // Read state
    Vector<double, N> state(dst_state_count);
    fragment_index_->copyFragments(slice.element_wise_begin(), slice.num_elements(),
                                   state.data(), state.size());
    // Add element to time series
    if (!series.tryPush(slice(*time_index), std::move(state))) {
      // Timestamps out of order
      return false;
    }
  }

  return true;
}

}  // namespace composite
}  // namespace isaac
