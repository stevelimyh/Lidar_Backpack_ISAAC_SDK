/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RepeatBehavior.hpp"

#include "engine/core/logger.hpp"

namespace isaac {
namespace behavior_tree {

void RepeatBehavior::start() {
  // In case there are no children the sequence succeeds.
  if (getNumChildren() != 1) {
    reportSuccess("Repeat behavior only works with exactly one child. Got %lld", getNumChildren());
    return;
  }
  tickOnChildStatus();
  // Start the one and only child
  startChild(0);
}

void RepeatBehavior::tick() {
  bool restart_child = false;
  // Check status of the current child and react accordingly
  switch (getChildStatus(0)) {
    case alice::Status::SUCCESS:
      restart_child = true;
      break;
    case alice::Status::FAILURE:
      if (get_repeat_after_failure()) {
        restart_child = true;
      } else {
        reportFailure("child reported failure");
      }
      break;
    case alice::Status::RUNNING:
      break;
    case alice::Status::INVALID:
      reportFailure("child with invalid status");
      return;
    default:
      PANIC("child with unknown status");
  }
  if (restart_child) {
    // TODO(dweikersdorf) This behavior should not block the thread with waiting.
    Sleep(SecondsToNano(get_wait_duration()));
    startChild(0);
  }
}

}  // namespace behavior_tree
}  // namespace isaac
