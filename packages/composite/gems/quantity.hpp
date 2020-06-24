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
#include <utility>

#include "engine/core/math/types.hpp"
#include "packages/composite/gems/measure.hpp"

namespace isaac {
namespace composite {

// A quantity describes meta data for values stored in a composite.
struct Quantity {
  // Helper function to create a scalar quantity
  static Quantity Scalar(std::string entity, Measure measure) {
    return Quantity{std::move(entity), measure, VectorXi::Constant(1, 1)};
  }

  // Helper function to create a vector quantity
  static Quantity Vector(std::string entity, Measure measure, int dimension) {
    return Quantity{std::move(entity), measure, VectorXi::Constant(1, dimension)};
  }

  // Total number of elements
  int getElementCount() const { return dimensions.prod(); }

  // The name of the entity for which the values was measured. Multiple quantities can be
  // measured per entity.
  std::string entity;
  // The measure, e.g. unit or type, of the value.
  Measure measure;
  // The dimensions of the value. Values can be multi-dimensional but most are scalars or vectors.
  VectorXi dimensions;
};

// Exact comparison of quantities
struct QuantityEquality {
  bool operator()(const Quantity& lhs, const Quantity& rhs) const {
    return lhs.measure == rhs.measure && lhs.entity == rhs.entity
        && lhs.dimensions == rhs.dimensions;
  }
};

// Hash function for quantities using combination of hashes on elements
struct QuantityHash {
  std::size_t operator()(const Quantity& value) const {
    return std::hash<std::string>{}.operator()(value.entity)
           ^ std::hash<int32_t>{}.operator()(static_cast<int32_t>(value.measure))
           ^ std::hash<int32_t>{}.operator()(static_cast<int32_t>(value.dimensions.size()));
  }
};

}  // namespace composite
}  // namespace isaac
