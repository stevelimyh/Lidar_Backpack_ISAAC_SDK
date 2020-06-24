/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace behavior_tree {
namespace deprecated {

class Behavior;

// @experimental
// Generalization of SelectorBehavior.
// While SelectorBehavior activates only one of all the nodes it has control over,
// GroupSelectorBehavior may activate a group of nodes. A map determines which nodes should be
// activated given desired behavior.
class GroupSelectorBehavior : public alice::Codelet {
 public:
  using map_behavior_t = std::map<std::string, std::vector<std::string>>;

  void start() override;
  void tick() override;
  void stop() override;

  // The desired behavior. Changing this live will trigger a behavior switch.
  ISAAC_PARAM(std::string, desired_behavior, "")
  // Map from behavior to list of nodes that needs to be active.
  ISAAC_PARAM(map_behavior_t, behavior_map, {});

 private:
  // Tries to stop the current behavior
  void stopCurrentBehavior();
  // Tries to starts a new desired behavior
  void startDesiredBehavior(const std::string& desired);

  map_behavior_t behavior_map_;
  Behavior* behavior_;
  std::string current_behavior_;
};

}  // namespace deprecated
}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::deprecated::GroupSelectorBehavior);
