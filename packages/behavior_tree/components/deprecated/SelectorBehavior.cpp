/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SelectorBehavior.hpp"

#include <string>

#include "packages/behavior_tree/components/deprecated/Behavior.hpp"

namespace isaac {
namespace behavior_tree {
namespace deprecated {

namespace {
constexpr const char* kNoBehavior = "";
}  // namespace

void SelectorBehavior::start() {
  behavior_ = node()->getComponent<Behavior>();
  current_behavior_ = kNoBehavior;
  tickPeriodically();
}

void SelectorBehavior::tick() {
  const std::string desired = get_desired_behavior();
  if (desired == current_behavior_) {
    return;
  }
  stopCurrentBehavior();
  startDesiredBehavior(desired);
}

void SelectorBehavior::stop() {
  stopCurrentBehavior();
}

void SelectorBehavior::stopCurrentBehavior() {
  if (current_behavior_ == kNoBehavior) {
    return;
  }
  behavior_->stop(current_behavior_);
  current_behavior_ = kNoBehavior;
}

void SelectorBehavior::startDesiredBehavior(const std::string& desired) {
  if (desired == kNoBehavior) {
    return;
  }
  ASSERT(current_behavior_ == kNoBehavior, "Must stop current behavior before starting another");
  if (behavior_->start(desired)) {
    current_behavior_ = desired;
  }
}

}  // namespace deprecated
}  // namespace behavior_tree
}  // namespace isaac
