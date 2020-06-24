/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "parser.hpp"

#include <algorithm>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace composite {

void Parser::requestSchema(Schema schema) {
  requested_schema_ = std::move(schema);
  setReceivedSchema(std::move(received_schema_));
}

void Parser::parseSchema(::CompositeProto::Reader reader) {
  const std::string incoming_schema_hash = reader.getSchemaHash();

  // Do not parse identical schema repeatedly
  if (!incoming_schema_hash.empty() && received_schema_ != std::nullopt
      && received_schema_->getHash() == incoming_schema_hash) {
    return;
  }

  // Parse received schema
  setReceivedSchema(ReadSchema(reader));
}

void Parser::setReceivedSchema(std::optional<Schema> schema) {
  received_schema_ = std::move(schema);

  if (received_schema_ != std::nullopt) {
    fragment_index_ = FragmentIndex::Create(*received_schema_, requested_schema_,
                                            requested_schema_);
  } else {
    fragment_index_ = std::nullopt;
  }
}

bool Parser::parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
                   double* output_begin, double* output_end) {
  // Fails on invalid output memory address
  if (output_begin == nullptr) {
    return false;
  }
  // Make sure element count matches
  const int output_element_count = output_end - output_begin;
  if (output_element_count != requested_schema_.getElementCount()) {
    return false;
  }
  // No parsing is needed
  if (output_element_count == 0) {
    return true;
  }

  // Parse the incoming schema
  parseSchema(reader);
  // Fail in case of incompatible schema
  if (fragment_index_ == std::nullopt) {
    return false;
  }

  // Get tensor storing quantity values
  CpuUniversalTensorConstView values;
  if (!FromProto(reader.getValues(), buffers, values)) {
    return false;
  }
  if (values.rank() == 0) {
    return false;
  }
  const int values_state_count = values.dimensions()[values.rank() - 1];
  if (values_state_count != received_schema_->getElementCount()) {
    return false;
  }
  if (values.element_type() != ElementType::kFloat64) {
    return false;
  }

  // Copy values from tensor to output buffer
  const double* values_ptr = reinterpret_cast<const double*>(values.buffer().begin());
  fragment_index_->copyFragments(values_ptr, values_state_count,
                                 output_begin, output_end - output_begin);

  return true;
}

}  // namespace composite
}  // namespace isaac
