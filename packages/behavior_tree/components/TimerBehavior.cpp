/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "TimerBehavior.hpp"

namespace isaac {
namespace behavior_tree {

void TimerBehavior::start() {
  tickPeriodically(get_delay());
}

void TimerBehavior::tick() {
  // tickPeriodically ticks immediately for the first time. However we want to change the status
  // only after the time delay, not immediately. Thus we ignore the first tick.
  if (isFirstTick()) {
    return;
  }
  updateStatus(get_status());
}

}  // namespace behavior_tree
}  // namespace isaac
