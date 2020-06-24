/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ParallelBehavior.hpp"

#include <string>

namespace isaac {
namespace behavior_tree {

void ParallelBehavior::start() {
  tickOnChildStatus();
  // Start all children
  for (size_t i = 0; i < getNumChildren(); i++) {
    startChild(i);
  }
}

void ParallelBehavior::tick() {
  // If there are no children we should not tick.
  const size_t num_children = getNumChildren();
  ASSERT(num_children != 0, "logic error");
  // Count status of children
  size_t num_success = 0;
  size_t num_failure = 0;
  for (size_t i = 0; i < num_children; i++) {
    switch (getChildStatus(i)) {
      case alice::Status::SUCCESS:
        num_success++;
        break;
      case alice::Status::FAILURE:
        num_failure++;
        break;
      case alice::Status::RUNNING:
        // not interested
        break;
      case alice::Status::INVALID:
        reportFailure("child with invalid status");
        return;
      default:
        PANIC("child with unknown status");
    }
  }
  // Check success condition
  const int success_threshold = get_success_threshold();
  if (success_threshold < -1) {
    reportFailure("Invalid success threshold");
    return;
  }
  if ((success_threshold == -1 && num_success == num_children) ||
      (num_success >= static_cast<size_t>(success_threshold))) {
    reportSuccess();
    return;
  }
  // Check failure condition
  const int failure_threshold = get_failure_threshold();
  if (failure_threshold < -1) {
    reportFailure("Invalid failure threshold");
    return;
  }
  if ((failure_threshold == -1 && num_failure == num_children) ||
      (num_failure >= static_cast<size_t>(failure_threshold))) {
    reportFailure();
    return;
  }
}

}  // namespace behavior_tree
}  // namespace isaac
