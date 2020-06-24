/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "GroupSelectorBehavior.hpp"

#include <map>
#include <string>
#include <vector>

#include "packages/behavior_tree/components/deprecated/Behavior.hpp"

namespace isaac {
namespace behavior_tree {
namespace deprecated {

namespace {
constexpr const char* kNoBehavior = "";
}  // namespace

void GroupSelectorBehavior::start() {
  behavior_ = node()->getComponent<Behavior>();
  current_behavior_ = kNoBehavior;
  behavior_map_ = get_behavior_map();
  tickPeriodically();
}

void GroupSelectorBehavior::tick() {
  const std::string desired = get_desired_behavior();
  if (desired == current_behavior_) {
    return;
  }
  stopCurrentBehavior();
  startDesiredBehavior(desired);
}

void GroupSelectorBehavior::stop() {
  stopCurrentBehavior();
}

void GroupSelectorBehavior::stopCurrentBehavior() {
  if (current_behavior_ == kNoBehavior) {
    return;
  }
  for (const auto& node_name : behavior_map_.at(current_behavior_)) {
    behavior_->stop(node_name);
  }
  current_behavior_ = kNoBehavior;
}

void GroupSelectorBehavior::startDesiredBehavior(const std::string& desired) {
  if (desired == kNoBehavior) {
    return;
  }
  ASSERT(current_behavior_ == kNoBehavior, "Must stop current behavior before starting another");
  if (behavior_map_.find(desired) == behavior_map_.end()) {
    LOG_ERROR("Unknown desired behavior '%s'.", desired.c_str());
    return;
  }

  for (const auto& node_name : behavior_map_.at(desired)) {
    behavior_->start(node_name);
  }
  current_behavior_ = desired;
}

}  // namespace deprecated
}  // namespace behavior_tree
}  // namespace isaac
