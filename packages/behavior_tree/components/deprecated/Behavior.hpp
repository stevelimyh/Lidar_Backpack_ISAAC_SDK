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
#include <unordered_map>
#include <vector>

#include "engine/alice/component.hpp"

namespace isaac {
namespace behavior_tree {
namespace deprecated {

class BehaviorBackend;

// @experimental
// Marks a node as a behavior so that it can be part of a behavior tree or state machine.
class Behavior : public alice::Component {
 public:
  void initialize() override;
  void deinitialize() override;
  // Gets the linked behavior with the given alias
  alice::Node* get(const std::string& alias);
  // Stops the linked behavior with given alias
  bool stop(const std::string& alias);
  // Starts the linked behavior with given alias
  bool start(const std::string& alias);

  // Names of nodes which will be controlled by this behavior
  ISAAC_PARAM(std::vector<std::string>, nodes, {});
  // Alias for linked nodes
  ISAAC_PARAM(std::vector<std::string>, aliases, {});

 private:
  // Parses nodes/aliases from configuration
  bool parseLinksFromConfig();

  std::unordered_map<std::string, alice::Node*> child_nodes_;
};

}  // namespace deprecated
}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::behavior_tree::deprecated::Behavior)
