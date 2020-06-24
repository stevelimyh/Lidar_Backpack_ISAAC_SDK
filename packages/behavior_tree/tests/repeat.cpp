/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"
#include "packages/behavior_tree/components/RepeatBehavior.hpp"
#include "packages/behavior_tree/tests/utils.hpp"

namespace isaac {
namespace behavior_tree {

class StartStopCounter : public alice::Codelet {
 public:
  void start() override {
    num_started++;
    tickPeriodically();
  }
  void tick() override {
    updateStatus(get_report_failure() ? alice::Status::FAILURE : alice::Status::SUCCESS);
  }
  void stop() override {
    num_stopped++;
  }

  ISAAC_PARAM(bool, report_failure, false);

  int num_started = 0;
  int num_stopped = 0;
};

TEST(RepeatBehavior, RepeatSuccess) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "parent", {"child"});
  repeat->async_set_wait_duration(0.1);
  auto* start_stop_counter = CreateSubBehaviorNode<StartStopCounter>(app, "child");
  start_stop_counter->async_set_tick_period("10Hz");
  app.startWaitStop(0.95);
  EXPECT_NEAR(start_stop_counter->num_started, 10, 1);
  EXPECT_NEAR(start_stop_counter->num_stopped, 10, 1);
}

TEST(RepeatBehavior, DoNotRepeatFailure) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "parent", {"child"});
  repeat->async_set_wait_duration(0.1);
  auto* start_stop_counter = CreateSubBehaviorNode<StartStopCounter>(app, "child");
  start_stop_counter->async_set_tick_period("10Hz");
  start_stop_counter->async_set_report_failure(true);
  app.startWaitStop(0.95);
  EXPECT_NEAR(start_stop_counter->num_started, 1, 1);
  EXPECT_NEAR(start_stop_counter->num_stopped, 1, 1);
}

TEST(RepeatBehavior, RepeatFailure) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "parent", {"child"});
  repeat->async_set_wait_duration(0.1);
  repeat->async_set_repeat_after_failure(true);
  auto* start_stop_counter = CreateSubBehaviorNode<StartStopCounter>(app, "child");
  start_stop_counter->async_set_tick_period("10Hz");
  start_stop_counter->async_set_report_failure(true);
  app.startWaitStop(0.95);
  EXPECT_NEAR(start_stop_counter->num_started, 10, 1);
  EXPECT_NEAR(start_stop_counter->num_stopped, 10, 1);
}

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::StartStopCounter);
