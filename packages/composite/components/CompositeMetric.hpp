/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/component.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

// Computes distance between two composite protos. The distance is determined by three factors:
// Schema: the schema used to parse values from the two composites. This schema must be a subset of
//         schemas in both protos
// Norm(s): for each quantity in the schema, the value is represented by a vector. The distance
//          between vectors depends on the norm used, for example L1, L2, ect. Norm is
//          specified via an double p, for p >=1 this represents the finite p-norm, while infinite
//          norm is represented by p = -1.0.
// Weight(s): the distance of each quantity is summed with weights to produce the total distance for
//            the two protos
class CompositeMetric : public alice::Component {
 public:
  // If allow_set_schema is true, sets the schema for distance calculation to the input variable
  // schema. Otherwise loads schema from config, ignores the input schema. In addition, loads
  // norms and weights from config. Validates the set of norms and weights are either 1, or
  // matches the number of quantities specified by schema.
  void setOrLoadSchema(const Schema* schema);
  // Returns the schema this component will used for distance calculation. Must be called
  // after setOrLoadSchema. Caller should  use getSchema() to get the schema, use it to parse the
  // state x and y before calling distance(x, y) below.
  const std::optional<composite::Schema> getSchema();
  // Calculates distance between two vectors representing the composite state. Size of the vectors
  // must match the element counts in the schema.
  std::optional<double> distance(const VectorXd& x, const VectorXd& y);

  // If true, the schema from the config below is used to compute distance. Otherwise, caller
  // should set the schema
  ISAAC_PARAM(bool, use_config_schema, false);
  // The schema to compute distance for. See json_formatter.hpp for json representation of Schema.
  // This is use only when use_config_schema is to True.
  ISAAC_PARAM(Schema, schema);
  // The list of p-norms to compute distance for each quantity in schema. The size should match
  // the list of quantities in schema, or one in which case it's applied to all quantities.
  ISAAC_PARAM(VectorXd, norms, VectorXd::Constant(1, 2.0));
  // Optional param to multiply with entity distance to get the total distance.
  ISAAC_PARAM(VectorXd, weights, VectorXd::Constant(1, 1.0));

 private:
  // Loads weights, metrics and schema from config. Returns false if any of these configs are
  // invalid.
  bool loadConfig();

  // If true, allow caller to set schema with trySetSchema. Otherwise load schema from config.
  bool allow_set_schema_;
  // Schema with all the quantities participating in distance computation.
  std::optional<composite::Schema> schema_;
  // Norms used to calculate distance.
  VectorXd norms_;
  // True if norms size is 1, e.g., it is uniformly applied to all quantities.
  bool uniform_norm_;
  // Weights used to calculate distance.
  VectorXd weights_;
  // True if weights size is 1, e.g., it is uniformly applied to all quantities.
  bool uniform_weight_;
};

}  // namespace composite
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::composite::CompositeMetric)
