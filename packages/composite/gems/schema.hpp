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
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "messages/composite.capnp.h"
#include "packages/composite/gems/quantity.hpp"

namespace isaac {
namespace composite {

// A schema contains information about the quantities which are stored in the composite
class Schema {
 public:
  // Constructs schema from given quanties
  static std::optional<Schema> Create(std::vector<Quantity>&& quantities, std::string hash = "");
  // Constructs schema from scalar quantities of identical measure
  static std::optional<Schema> Create(const std::vector<std::string>& entities,
                                      const Measure measure);

  // Similar to `Create` but will Assert if schema is invalid
  Schema(std::vector<Quantity>&& quantities, std::string hash = "");
  // Similar to `Create` but will Assert if schema is invalid
  Schema(const std::vector<std::string>& entities, const Measure measure);

  // Empty schema
  Schema() = default;

  // The hash of the schame
  const std::string& getHash() const { return hash_; }

  // Returns a list of all quantities
  const std::vector<Quantity>& getQuantities() const { return quantities_; }

  // Total number of values required to store the schema
  int getElementCount() const { return element_count_; }

  // Gets the starting index of a quantity in the list ofvalues. Returns nullopt if quantity is
  // not in the schema.
  std::optional<int> findQuantityValueIndex(const Quantity& quantity) const;

 private:
  // Updates `element_count_`, `values_offset_` and `index_` variables based on quantities.
  bool createIndex();

  // The hash of the currently used schema
  std::string hash_;

  // All quantities which are part of this schema
  std::vector<Quantity> quantities_;

  // Total number of elements in the schema
  int element_count_ = 0;

  // Indies of first quantity element in the values vector
  std::unordered_map<Quantity, int, QuantityHash, QuantityEquality> index_;
};

// Reads a schema from a composite proto
std::optional<Schema> ReadSchema(::CompositeProto::Reader reader);

// Writes a schema to a composite proto
void WriteSchema(const Schema& schema, ::CompositeProto::Builder writer);

}  // namespace composite
}  // namespace isaac
