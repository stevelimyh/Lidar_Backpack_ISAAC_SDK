/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace behavior_tree {
namespace deprecated {

class Behavior;

// @experimental
// A basic behavior which runs one of its child behaviors.
class SelectorBehavior : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // The desired behavior. Changing this live will trigger a behavior switch.
  ISAAC_PARAM(std::string, desired_behavior, "")

 private:
  // Tries to stop the current behavior
  void stopCurrentBehavior();
  // Tries to starts a new desired behavior
  void startDesiredBehavior(const std::string& desired);

  Behavior* behavior_;
  std::string current_behavior_;
};

}  // namespace deprecated
}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::deprecated::SelectorBehavior);
