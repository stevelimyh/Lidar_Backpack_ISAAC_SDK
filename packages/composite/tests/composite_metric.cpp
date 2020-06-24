/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/composite/components/CompositeMetric.hpp"

#include <random>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace composite {

// Checks if the received goal is equal to the given goal
class MetricConfigCheck : public alice::Codelet {
 public:
  void start() override {
    metric_ = node()->getComponent<CompositeMetric>();
    metric_->setOrLoadSchema(nullptr);
    EXPECT_TRUE(metric_->getSchema());
    Vector7d x, y;
    x << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // rotation is {0.5, {1.0, 1.0, 1.0}} (angle-axis)
    y << 0.9689124, 0.1428387, 0.1428387, 0.1428387, 3.0, 0.0, 4.0;
    const auto may_d = metric_->distance(x, y);
    EXPECT_TRUE(may_d);
    EXPECT_NEAR(double(10.0), *may_d, 1e-5);
  }

 private:
  CompositeMetric* metric_;
};

// Checks if the received goal is equal to the given goal
class MetricSetterCheck : public alice::Codelet {
 public:
  void start() override {
    metric_ = node()->getComponent<CompositeMetric>();

    const int count = 9;
    const auto schema = Schema::Create({Quantity::Vector("ee", Measure::kSpeed, count)});

    metric_->setOrLoadSchema(&(*schema));
    const auto metric_schema = metric_->getSchema();
    EXPECT_TRUE(metric_schema);
    EXPECT_EQ(count, metric_schema->getElementCount());

    const VectorXd x = VectorXd::Random(count);
    const VectorXd y = VectorXd::Random(count);
    const double delta = (x - y).cwiseAbs().maxCoeff();
    const auto may_d = metric_->distance(x, y);
    EXPECT_TRUE(may_d);
    EXPECT_NEAR(delta, *may_d, 1e-7);
  }

 private:
  CompositeMetric* metric_;
};

TEST(CompositeMetricConfig, Test) {
  alice::Application app;

  auto* metric_node = app.createMessageNode("metric");
  auto* composite_metric = metric_node->addComponent<CompositeMetric>("CompositeMetric");
  metric_node->addComponent<MetricConfigCheck>("MetricConfigCheck");

  composite_metric->async_set_use_config_schema(true);
  const auto schema = Schema::Create({Quantity::Vector("ee", Measure::kRotation, 4),
                                      Quantity::Vector("ee", Measure::kPosition, 3)});
  composite_metric->async_set_schema(*schema);
  composite_metric->async_set_weights(Vector2d{10.0, 1.0});
  app.startWaitStop(0.50);
}

TEST(CompositeMetricSetter, Test) {
  alice::Application app;

  auto* metric_node = app.createMessageNode("metric");
  auto* composite_metric = metric_node->addComponent<CompositeMetric>("CompositeMetric");
  metric_node->addComponent<MetricSetterCheck>("MetricSetterCheck");

  composite_metric->async_set_use_config_schema(false);
  composite_metric->async_set_norms(VectorXd::Constant(1, -1.0));
  app.startWaitStop(0.50);
}

}  // namespace composite
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::composite::MetricConfigCheck);
ISAAC_ALICE_REGISTER_CODELET(isaac::composite::MetricSetterCheck);
