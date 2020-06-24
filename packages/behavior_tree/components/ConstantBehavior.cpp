/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ConstantBehavior.hpp"

namespace isaac {
namespace behavior_tree {

void ConstantBehavior::start() {
  const auto status = get_status();
  if (status != alice::Status::RUNNING) {
    updateStatus(status);
  }

  // Tick if user set tick_period
  if (try_get_tick_period()) {
    tickPeriodically();
  }
}

void ConstantBehavior::tick() {
  switch (get_status()) {
    case alice::Status::SUCCESS:
      reportSuccess();
      return;
    case alice::Status::FAILURE:
      reportFailure();
      return;
    default:
      // Do nothing for Status::RUNNING.
      // Ignore Status::INVALID because:
      // 1. We report failure only when status parameter is set to Status::FAILURE.
      // 2. status parameter may be set through Sight. Do not assert if user makes a typo.
      return;
  }
}

}  // namespace behavior_tree
}  // namespace isaac
