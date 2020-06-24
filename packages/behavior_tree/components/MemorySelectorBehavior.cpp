/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MemorySelectorBehavior.hpp"

#include "engine/core/logger.hpp"

namespace isaac {
namespace behavior_tree {

void MemorySelectorBehavior::start() {
  // In case there are no children the selector fails.
  if (getNumChildren() == 0) {
    reportFailure("selector without children");
    return;
  }
  tickOnChildStatus();
  // Start the first child
  current_child_index_ = 0;
  startChild(current_child_index_);
}

void MemorySelectorBehavior::tick() {
  // As we are ticking only when a child changes its status we should not tick after we reached
  // the last child.
  if (current_child_index_ >= getNumChildren()) {
    LOG_WARNING("Parent behavior was ticked although all children terminated");
    return;
  }
  // Check status of the current child and react if it terminated
  switch (getChildStatus(current_child_index_)) {
    case alice::Status::SUCCESS:
      // A child which succeeds was found thus we are done.
      reportSuccess();
      return;
    case alice::Status::FAILURE:
      // The current child failed thus we go to next child.
      current_child_index_++;
      // If we went through all children without finding a successful one we fail, otherwise we try
      // the next child.
      if (current_child_index_ >= getNumChildren()) {
        reportFailure("all children failed");
      } else {
        startChild(current_child_index_);
      }
      return;
    case alice::Status::RUNNING:
      // keep running
      return;
    case alice::Status::INVALID:
      reportFailure("child with invalid status");
      return;
    default:
      PANIC("child with unknown status");
  }
}

}  // namespace behavior_tree
}  // namespace isaac
