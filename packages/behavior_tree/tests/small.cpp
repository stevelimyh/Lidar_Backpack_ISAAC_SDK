/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/behavior_tree/components/ConstantBehavior.hpp"
#include "packages/behavior_tree/components/MemorySelectorBehavior.hpp"
#include "packages/behavior_tree/components/MemorySequenceBehavior.hpp"
#include "packages/behavior_tree/components/NodeGroup.hpp"
#include "packages/behavior_tree/components/ParallelBehavior.hpp"
#include "packages/behavior_tree/components/SwitchBehavior.hpp"
#include "packages/behavior_tree/components/TimerBehavior.hpp"
#include "packages/behavior_tree/tests/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace behavior_tree {

namespace {

// Tests start() of ConstantBehavior. tick_period is not set.
void CreateConstantBehaviorStartTest(alice::Status status) {
  alice::Application app;
  alice::Node* node = app.createNode("behavior");
  auto* behavior = node->addComponent<ConstantBehavior>();
  behavior->async_set_status(status);
  app.startWaitStop(0.10);
  EXPECT_EQ(behavior->getStatus(), status);
}

// Tests tick() of ConstantBehavior. Requires tick_period to be set.
void CreateConstantBehaviorTickTest(alice::Status status) {
  alice::Application app;
  alice::Node* node = app.createNode("behavior");
  auto* behavior = node->addComponent<ConstantBehavior>();
  behavior->async_set_tick_period("0.01s");
  app.start();
  EXPECT_EQ(behavior->getStatus(), alice::Status::RUNNING);
  behavior->async_set_status(status);
  Sleep(SecondsToNano(0.10));
  app.stop();
  EXPECT_EQ(behavior->getStatus(), status);
}

void CreateTimerBehaviorTest(double delay, alice::Status status, double app_duration,
                             alice::Status expected_status) {
  alice::Application app;
  alice::Node* node = app.createNode("behavior");
  auto* behavior = node->addComponent<TimerBehavior>();
  behavior->async_set_delay(delay);
  behavior->async_set_status(status);
  app.startWaitStop(app_duration);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

template <typename Behavior>
void CreateBehaviorTest(const std::vector<alice::Status>& child_stati,
                        alice::Status expected_status) {
  std::vector<std::string> names;
  for (size_t i = 0; i < child_stati.size(); i++) {
    names.push_back("child_" + std::to_string(i));
  }
  alice::Application app;
  auto* behavior = CreateCompositeBehaviorNode<Behavior>(app, "parent", names);
  for (size_t i = 0; i < child_stati.size(); i++) {
    CreateConstantBehaviorNode(app, names[i], child_stati[i]);
  }
  app.startWaitStop(0.10);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

template <typename Behavior>
void CreateBehaviorTest(const std::vector<std::pair<double, alice::Status>>& child_data,
                        alice::Status expected_status, double duration) {
  std::vector<std::string> names;
  for (size_t i = 0; i < child_data.size(); i++) {
    names.push_back("child_" + std::to_string(i));
  }
  alice::Application app;
  auto* behavior = CreateCompositeBehaviorNode<Behavior>(app, "parent", names);
  for (size_t i = 0; i < child_data.size(); i++) {
    CreateTimerBehaviorNode(app, names[i], child_data[i].first, child_data[i].second);
  }
  app.startWaitStop(duration);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

}  // namespace

TEST(Behaviors, ConstantBehavior) {
  CreateConstantBehaviorStartTest(alice::Status::SUCCESS);
  CreateConstantBehaviorStartTest(alice::Status::FAILURE);
  CreateConstantBehaviorStartTest(alice::Status::RUNNING);
}

TEST(Behaviors, ParameterizedBehavior) {
  CreateConstantBehaviorTickTest(alice::Status::SUCCESS);
  CreateConstantBehaviorTickTest(alice::Status::FAILURE);
  CreateConstantBehaviorTickTest(alice::Status::RUNNING);
}

TEST(Behaviors, TimerBehavior) {
  CreateTimerBehaviorTest(0.05, alice::Status::SUCCESS, 0.15, alice::Status::SUCCESS);
  CreateTimerBehaviorTest(0.05, alice::Status::FAILURE, 0.15, alice::Status::FAILURE);
  CreateTimerBehaviorTest(10.0, alice::Status::FAILURE, 0.15, alice::Status::RUNNING);
}

TEST(Behaviors, MemorySelectorBehavior) {
  CreateBehaviorTest<MemorySelectorBehavior>({alice::Status::SUCCESS}, alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySelectorBehavior>({alice::Status::FAILURE}, alice::Status::FAILURE);
  CreateBehaviorTest<MemorySelectorBehavior>({alice::Status::RUNNING}, alice::Status::RUNNING);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::RUNNING, alice::Status::SUCCESS}, alice::Status::RUNNING);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::FAILURE, alice::Status::SUCCESS}, alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::FAILURE, alice::Status::FAILURE, alice::Status::SUCCESS},
      alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::SUCCESS, alice::Status::SUCCESS, alice::Status::FAILURE},
      alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::SUCCESS, alice::Status::SUCCESS, alice::Status::SUCCESS},
      alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySelectorBehavior>(
      {alice::Status::FAILURE, alice::Status::FAILURE, alice::Status::FAILURE},
      alice::Status::FAILURE);
}

TEST(Behaviors, MemorySequenceBehavior) {
  CreateBehaviorTest<MemorySequenceBehavior>({alice::Status::SUCCESS}, alice::Status::SUCCESS);
  CreateBehaviorTest<MemorySequenceBehavior>({alice::Status::FAILURE}, alice::Status::FAILURE);
  CreateBehaviorTest<MemorySequenceBehavior>({alice::Status::RUNNING}, alice::Status::RUNNING);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::RUNNING, alice::Status::SUCCESS}, alice::Status::RUNNING);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::FAILURE, alice::Status::SUCCESS}, alice::Status::FAILURE);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::FAILURE, alice::Status::FAILURE, alice::Status::SUCCESS},
      alice::Status::FAILURE);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::SUCCESS, alice::Status::FAILURE, alice::Status::SUCCESS},
      alice::Status::FAILURE);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::SUCCESS, alice::Status::SUCCESS, alice::Status::FAILURE},
      alice::Status::FAILURE);
  CreateBehaviorTest<MemorySequenceBehavior>(
      {alice::Status::SUCCESS, alice::Status::SUCCESS, alice::Status::SUCCESS},
      alice::Status::SUCCESS);
}

TEST(Behaviors, ParallelBehavior) {
  CreateBehaviorTest<ParallelBehavior>(
      {{0.05, alice::Status::SUCCESS}, {0.10, alice::Status::SUCCESS}},
      alice::Status::SUCCESS, 0.15);
  CreateBehaviorTest<ParallelBehavior>(
      {{0.05, alice::Status::SUCCESS}, {0.10, alice::Status::FAILURE}},
      alice::Status::FAILURE, 0.15);
  CreateBehaviorTest<ParallelBehavior>(
      {{0.05, alice::Status::SUCCESS}, {10.0, alice::Status::FAILURE}},
      alice::Status::RUNNING, 0.10);
  CreateBehaviorTest<ParallelBehavior>(
      {{0.05, alice::Status::SUCCESS}, {0.03, alice::Status::SUCCESS},
       {0.05, alice::Status::SUCCESS}, {0.06, alice::Status::SUCCESS},
       {0.09, alice::Status::SUCCESS}, {0.04, alice::Status::SUCCESS}},
      alice::Status::SUCCESS, 0.15);
}

TEST(Behaviors, SwitchBehavior) {
  alice::Application app;
  auto* behavior = CreateCompositeBehaviorNode<SwitchBehavior>(app, "parent",
      {"child_0", "child_1", "child_2"});
  CreateTimerBehaviorNode(app, "child_0", 0.10, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "child_1", 0.20, alice::Status::FAILURE);
  CreateTimerBehaviorNode(app, "child_2", 0.30, alice::Status::SUCCESS);
  app.start();
  Sleep(0.20);
  behavior->async_set_desired_behavior("child_1");
  Sleep(0.10);
  behavior->async_set_desired_behavior("child_0");
  Sleep(0.20);
  app.stop();
  EXPECT_EQ(behavior->getStatus(), alice::Status::RUNNING);
}

TEST(Behaviors, Nested) {
  alice::Application app;

  auto* top = CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "top",
      {"0", "3", "1", "4", "2"});
  EXPECT_EQ(top->getStatus(), alice::Status::RUNNING);

  CreateCompositeBehaviorNode<MemorySelectorBehavior>(app, "0", {"00", "01", "02"});
  CreateTimerBehaviorNode(app, "00", 0.07, alice::Status::FAILURE);
  CreateTimerBehaviorNode(app, "01", 0.06, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "02", 0.09, alice::Status::FAILURE);

  CreateCompositeBehaviorNode<MemorySelectorBehavior>(app, "3", {"30", "31", "32"});
  CreateTimerBehaviorNode(app, "30", 0.07, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "31", 0.11, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "32", 0.05, alice::Status::SUCCESS);

  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "1", {"10", "11", "12"});
  CreateTimerBehaviorNode(app, "10", 0.04, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "11", 0.05, alice::Status::SUCCESS);
  CreateTimerBehaviorNode(app, "12", 0.06, alice::Status::SUCCESS);

  CreateTimerBehaviorNode(app, "4", 0.06, alice::Status::SUCCESS);

  CreateCompositeBehaviorNode<MemorySelectorBehavior>(app, "2", {"20", "21", "22"});
  CreateTimerBehaviorNode(app, "20", 0.03, alice::Status::FAILURE);
  CreateTimerBehaviorNode(app, "21", 0.08, alice::Status::FAILURE);
  CreateTimerBehaviorNode(app, "22", 0.05, alice::Status::SUCCESS);

  app.startWaitStop(1.50);
  EXPECT_EQ(top->getStatus(), alice::Status::SUCCESS);
}

}  // namespace behavior_tree
}  // namespace isaac

