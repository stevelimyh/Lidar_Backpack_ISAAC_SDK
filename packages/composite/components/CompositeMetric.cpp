/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CompositeMetric.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <cmath>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace composite {

namespace {

// Calculates the p-norm of a vector. p must be >=1 or -1 for infinite norm.
double PNorm(const VectorXd& x, double p) {
  if (IsAlmostOne(-p)) {
    // infinite norm
    return x.lpNorm<Eigen::Infinity>();
  } else if (p >= 1) {
    return std::pow(Eigen::pow(x.cwiseAbs().array(), p).sum(), 1.0 / p);
  } else {
    PANIC("%f-norm not supported. p must be >=1 or -1.", p);
  }
}

}  // namespace

void CompositeMetric::setOrLoadSchema(const Schema* schema) {
  if (get_use_config_schema()) {
    // load schema
    schema_ = try_get_schema();
    ASSERT(schema_, "Fails to load schema from config.");
  } else {
    ASSERT(schema, "Input schema must be valid for setSchema.");
    std::vector<Quantity> quantities = schema->getQuantities();
    schema_ = Schema::Create(std::move(quantities), schema_->getHash());
    ASSERT(schema_, "Fails to set schema from input.");
  }
  // load metrics and weight from config
  weights_ = get_weights();
  uniform_weight_ = (weights_.size() == 1);
  norms_ = get_norms();
  uniform_norm_ = (norms_.size() == 1);
  if (!uniform_weight_ && !uniform_norm_) {
    ISAAC_ASSERT_EQ(norms_.size(), weights_.size());
  }
  // validate number of quantities in schema and metrics/weights match
  const int count = schema_->getQuantities().size();
  if (!uniform_weight_) {
    ISAAC_ASSERT_EQ(weights_.size(), count);
  }
  if (!uniform_norm_) {
    ISAAC_ASSERT_EQ(norms_.size(), count);
  }
  // validate p-norm values is >=1 or ==-1
  for (int i = 0; i < norms_.size(); i++) {
    const double p = norms_[i];
    ASSERT(p >= 1 || IsAlmostOne(-p), "%f-norm not supported. p must be >=1 or -1.", p);
  }
}

const std::optional<composite::Schema> CompositeMetric::getSchema() {
  ASSERT(schema_, "getSchema called before schema is set.");
  return schema_;
}

std::optional<double> CompositeMetric::distance(const VectorXd& x, const VectorXd& y) {
  ASSERT(schema_, "distance called before schema is set.");
  const int values_size = schema_->getElementCount();
  if (x.size() != values_size || y.size() != values_size) {
    return std::nullopt;
  }
  // get distance for each quantity
  const auto& quantities = schema_->getQuantities();
  const int quantities_size = quantities.size();
  VectorXd distances(quantities_size);
  int offset = 0;
  for (int i = 0; i < quantities_size; i++) {
    const auto& quantity = quantities[i];
    const int n = quantity.dimensions.prod();
    if (n == 0) {
      // for scalar, return absolute value (independent of p)
      distances[i] = std::abs(x[offset] - y[offset]);
    } else {
      // for vectors
      const VectorXd px = x.segment(offset, n);
      const VectorXd py = y.segment(offset, n);
      const double norm = uniform_norm_ ? norms_[0] : norms_[i];
      if (quantities[i].measure != Measure::kRotation) {
        // handle non-rotation quantity as vector
        distances[i] = PNorm(px - py, norm);
      } else {
        // handle rotation separately.
        if (quantity.dimensions.size() == 1) {
          // Skip 1x1 (angle) which is already handled in n=1 scalar case.
          if (n == 2) {
            // 2x1, SO2
            const SO2<double> rx = SO2<double>::FromNormalized(px[0], px[1]);
            const SO2<double> ry = SO2<double>::FromNormalized(py[0], py[1]);
            distances[i] = std::abs((rx.inverse() * ry).angle());
          } else if (n == 3) {
            // 3x1 (scaled axis)
            distances[i] = PNorm(px - py, norm);
          } else if (n == 4) {
            // 4x1, SO3 quaternion
            Quaterniond qx(px[0], px[1], px[2], px[3]), qy(py[0], py[1], py[2], py[3]);
            distances[i] = qx.angularDistance(qy);
          } else {
            PANIC("Rotation quantity vector size %d not supported.", n);
          }
        } else {
          // TODO(qianl): support rotation matrix (2x2 and 3x3)
          PANIC("Rotation quantity must be a vector. %zu dimensions not supported.",
                quantity.dimensions.size());
        }
      }
    }
    offset += n;
  }
  // get weighted total distance
  if (uniform_weight_) {
    return distances.sum() * weights_[0];
  } else {
    return distances.dot(weights_);
  }
}

}  // namespace composite
}  // namespace isaac
