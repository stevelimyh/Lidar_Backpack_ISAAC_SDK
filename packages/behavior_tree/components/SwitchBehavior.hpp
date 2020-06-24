/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/core/optional.hpp"
#include "packages/behavior_tree/components/Behavior.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// A behavior which executes exactly one of its children, selected by name.
class SwitchBehavior : public Behavior {
 public:
  void start() override;
  void tick() override;

  // The name of the child node with the desired behavior. Only the initial value is used, i.e.,
  // switching while this behavior is already running is not supported.
  ISAAC_PARAM(std::string, desired_behavior);

 private:
  // The behavior which is currently running
  std::optional<std::string> current_behavior_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::SwitchBehavior);
