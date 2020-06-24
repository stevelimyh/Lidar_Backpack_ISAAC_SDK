/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "schema.hpp"

#include <string>
#include <utility>
#include <vector>

#include "messages/element_type.hpp"
#include "messages/math.hpp"
#include "packages/composite/gems/measure.hpp"

namespace isaac {
namespace composite {

namespace {

// Parses quantity dimensions and uses scalar if not set
VectorXi ParseQuantityDimensions(::CompositeProto::Quantity::Reader reader) {
  if (reader.hasDimensions()) {
    return ::isaac::FromProto(reader.getDimensions());
  } else {
    return VectorXi::Constant(1, 1);
  }
}

}  // namespace

std::optional<Schema> Schema::Create(std::vector<Quantity>&& quantities, std::string hash) {
  Schema result;
  result.quantities_ = std::move(quantities);
  result.hash_ = std::move(hash);
  if (!result.createIndex()) {
    return std::nullopt;
  }
  return result;
}

std::optional<Schema> Schema::Create(const std::vector<std::string>& entities,
                                     const Measure measure) {
  Schema result;
  result.quantities_.reserve(entities.size());
  for (const std::string& entity : entities) {
    result.quantities_.push_back({entity, measure, VectorXi::Constant(1, 1)});
  }
  if (!result.createIndex()) {
    return std::nullopt;
  }
  return result;
}

Schema::Schema(std::vector<Quantity>&& quantities, std::string hash) {
  const auto schema = Create(std::move(quantities), std::move(hash));
  ASSERT(schema != std::nullopt, "invalid schema");
  *this = std::move(*schema);
}

Schema::Schema(const std::vector<std::string>& entities, const Measure measure) {
  const auto schema = Create(entities, measure);
  ASSERT(schema != std::nullopt, "invalid schema");
  *this = std::move(*schema);
}

std::optional<int> Schema::findQuantityValueIndex(const Quantity& quantity) const {
  const auto it = index_.find(quantity);
  if (it == index_.end()) {
    return std::nullopt;
  } else {
    return it->second;
  }
}

bool Schema::createIndex() {
  element_count_ = 0;
  index_.clear();
  for (const Quantity& quantity : quantities_) {
    if (index_.find(quantity) != index_.end()) {
      return false;
    }
    index_[quantity] = element_count_;
    element_count_ += quantity.getElementCount();
  }
  return true;
}

std::optional<Schema> ReadSchema(::CompositeProto::Reader reader) {
  auto quantities_proto = reader.getQuantities();
  std::vector<Quantity> quantities;
  quantities.reserve(quantities_proto.size());
  for (auto quantity_proto : quantities_proto) {
    quantities.push_back(Quantity{std::string(quantity_proto.getEntity()),
                                  FromProto(quantity_proto.getMeasure()),
                                  ParseQuantityDimensions(quantity_proto)});
  }
  return Schema::Create(std::move(quantities), reader.getSchemaHash());
}

void WriteSchema(const Schema& schema, ::CompositeProto::Builder writer) {
  const auto& quantities = schema.getQuantities();
  auto quantities_proto = writer.initQuantities(quantities.size());
  for (size_t index = 0; index < quantities.size(); index++) {
    const Quantity& quantity = quantities[index];
    quantities_proto[index].setEntity(quantity.entity);
    quantities_proto[index].setElementType(::ToProto(ElementType::kFloat64));
    quantities_proto[index].setMeasure(ToProto(quantity.measure));
    ::isaac::ToProto(quantity.dimensions, quantities_proto[index].initDimensions());
  }
  writer.setSchemaHash("");  // FIXME(yangl) implement
}

}  // namespace composite
}  // namespace isaac
