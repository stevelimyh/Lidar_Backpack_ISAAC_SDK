/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SwitchBehavior.hpp"

#include <string>

namespace isaac {
namespace behavior_tree {

void SwitchBehavior::start() {
  // Try to start the desired behavior
  current_behavior_ = try_get_desired_behavior();
  if (!current_behavior_) {  // no behavior is selected, then we already succeed
    reportSuccess();
  } else if (current_behavior_->empty()) {
    reportFailure("Empty string for desired behavior");
  } else {
    alice::Node* child = findChildByName(*current_behavior_);
    if (!child) {
      reportFailure("No child with name '%s'", current_behavior_->c_str());
    } else {
      startChild(*child);
    }
  }
  // FIXME: Ideally we would tick also if the configuration changes so we can switch behavior
  //  while running.
  tickOnChildStatus();
}

void SwitchBehavior::tick() {
  const auto status = getChildStatus(*current_behavior_);
  if (status != alice::Status::RUNNING) {
    updateStatus(status, "child status changed");
  }
}

}  // namespace behavior_tree
}  // namespace isaac
