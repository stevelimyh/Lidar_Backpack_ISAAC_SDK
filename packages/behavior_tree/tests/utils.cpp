/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "utils.hpp"

#include <string>

#include "packages/behavior_tree/components/ConstantBehavior.hpp"
#include "packages/behavior_tree/components/TimerBehavior.hpp"

namespace isaac {
namespace behavior_tree {

ConstantBehavior* CreateConstantBehaviorNode(alice::Application& app, const std::string& name,
                                             alice::Status status) {
  auto* behavior = CreateSubBehaviorNode<ConstantBehavior>(app, name);
  behavior->async_set_status(status);
  return behavior;
}

TimerBehavior* CreateTimerBehaviorNode(alice::Application& app, const std::string& name,
                                       double delay, alice::Status status) {
  auto* behavior = CreateSubBehaviorNode<TimerBehavior>(app, name);
  behavior->async_set_delay(delay);
  behavior->async_set_status(status);
  return behavior;
}

}  // namespace behavior_tree
}  // namespace isaac

