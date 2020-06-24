/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"
#include "packages/behavior_tree/components/MemorySequenceBehavior.hpp"
#include "packages/behavior_tree/components/ParallelBehavior.hpp"
#include "packages/behavior_tree/components/RepeatBehavior.hpp"
#include "packages/behavior_tree/tests/utils.hpp"

namespace isaac {
namespace behavior_tree {

namespace {
// Global variable to test whether nodes start and stop in expected order
int step = 0;
// Name to be used for stopwatches
constexpr char kStopwatchName[] = "stopwatch";
}  // namespace

class OrderChecker : public alice::Codelet {
 public:
  void start() override {
    ASSERT_EQ(step, 0);
    step = 1;
    tickPeriodically();
  }
  void tick() override {
    ASSERT_EQ(step, 1);
    step = 2;
    reportSuccess();
  }
  void stop() override {
    ASSERT_EQ(step, 2);
    step = 0;
  }
};

// Checks time between stop() calls
class TimeChecker : public alice::Codelet {
 public:
  void stop() override {
    if (stopwatch(kStopwatchName).running()) {
      stopwatch(kStopwatchName).stop();
      EXPECT_GE(stopwatch(kStopwatchName).read(), get_min_time_before_next_stop());
    }
    stopwatch(kStopwatchName).start();
  }

  // Time between two stop() calls need to be at list this many seconds apart
  ISAAC_PARAM(double, min_time_before_next_stop, 0.5);
};

// Run this test with:
//        bazel test packages/behavior_tree/tests:combined --test_output=all
//              --test_arg=--gtest_filter=*StartStopOrder
TEST(CombinedBehaviors, StartStopOrder) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"sequence"});
  repeat->async_set_wait_duration(0.1);

  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "sequence", {"first", "second"}, true);

  alice::Node* first = app.createNode("first");
  first->disable_automatic_start = true;
  first->addComponent<OrderChecker>()->async_set_tick_period("10Hz");

  alice::Node* second = app.createNode("second");
  second->disable_automatic_start = true;
  second->addComponent<OrderChecker>()->async_set_tick_period("10Hz");

  app.startWaitStop(0.95);
}

// Run this test with:
//        bazel test packages/behavior_tree/tests:combined --test_output=all
//              --test_arg=--gtest_filter=*RepeatSequenceOfParallels
TEST(CombinedBehaviors, RepeatSequenceOfParallels) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"sequence"});
  repeat->async_set_wait_duration(0.1);

  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "sequence", {"parallel_1", "parallel_2"},
                                                      true);

  for (int idx = 1; idx <= 2; ++idx) {
    // Names of nodes
    const std::string node_name_postfix = std::to_string(idx);
    const std::string node_name_parallel = "parallel_" + node_name_postfix;
    const std::string node_name_timer = "timer_" + node_name_postfix;
    const std::string node_name_constant = "constant_" + node_name_postfix;
    const std::string node_name_checker = "checker_" + node_name_postfix;

    auto* parallel = CreateCompositeBehaviorNode<ParallelBehavior>(
        app, node_name_parallel, {node_name_timer, node_name_constant, node_name_checker}, true);
    parallel->async_set_success_threshold(1);

    alice::Node* timer = app.createNode(node_name_timer);
    timer->disable_automatic_start = true;
    timer->addComponent<TimerBehavior>();

    alice::Node* constant = app.createNode(node_name_constant);
    constant->disable_automatic_start = true;
    constant->addComponent<ConstantBehavior>();

    alice::Node* checker = app.createNode(node_name_checker);
    checker->disable_automatic_start = true;
    checker->addComponent<TimeChecker>();
  }

  app.startWaitStop(6.00);
}

// Run this test with:
//      bazel test packages/behavior_tree/tests:combined --runs_per_test=10 --jobs 1
//          --test_arg=--gtest_filter=*RepeatSuccess
TEST(CombinedBehaviors, RepeatSuccess) {
  constexpr double kWaitBeforeRepeat = 1e-11;

  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"constant"});
  repeat->async_set_wait_duration(kWaitBeforeRepeat);

  alice::Node* constant = app.createNode("constant");
  constant->disable_automatic_start = true;
  constant->addComponent<ConstantBehavior>()->async_set_status(alice::Status::SUCCESS);

  app.startWaitStop(1.00);
  EXPECT_GE(repeat->getTickCount(), 3);
}

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::OrderChecker);
ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::TimeChecker);
