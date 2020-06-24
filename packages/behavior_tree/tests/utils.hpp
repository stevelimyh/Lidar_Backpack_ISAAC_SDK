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
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/behavior_tree/components/ConstantBehavior.hpp"
#include "packages/behavior_tree/components/NodeGroup.hpp"
#include "packages/behavior_tree/components/TimerBehavior.hpp"

namespace isaac {
namespace behavior_tree {

// Creates a group node with a behavior of given type
template <typename Behavior>
Behavior* CreateSubBehaviorNode(alice::Application& app, const std::string& name) {
  alice::Node* node = app.createNode(name);
  node->disable_automatic_start = true;
  return node->addComponent<Behavior>();
}

// Creates a group node with a behavior of given type
template <typename Behavior>
Behavior* CreateCompositeBehaviorNode(alice::Application& app, const std::string& name,
                                      const std::vector<std::string>& children,
                                      bool disable_automatic_start = false) {
  alice::Node* node = app.createNode(name);
  node->disable_automatic_start = disable_automatic_start;
  auto* node_group = node->addComponent<NodeGroup>();
  node_group->async_set_node_names(children);
  return node->addComponent<Behavior>();
}

// Creates a node with a behavior returning the given status
ConstantBehavior* CreateConstantBehaviorNode(alice::Application& app, const std::string& name,
                                             alice::Status status);

// Creates a node with a timer behavior
TimerBehavior* CreateTimerBehaviorNode(alice::Application& app, const std::string& name,
                                       double delay, alice::Status status);

}  // namespace behavior_tree
}  // namespace isaac

