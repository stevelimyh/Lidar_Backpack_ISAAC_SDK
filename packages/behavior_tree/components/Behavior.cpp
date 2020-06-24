/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Behavior.hpp"

#include <string>
#include <unordered_set>

#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "packages/behavior_tree/components/Behavior.hpp"
#include "packages/behavior_tree/components/NodeGroup.hpp"

namespace isaac {
namespace behavior_tree {

namespace {

// Channel name used for status updates
const char kStatusEventChannel[] = "__status";

}  // namespace

void Behavior::stop() {
  for (size_t i = 0; i < getNumChildren(); i++) {
    alice::Node& node = getChildByIndex(i);
    if (getChildStatus(node) == alice::Status::RUNNING) {
      stopChild(node);
    }
  }
}

void Behavior::tickOnChildStatus() {
  std::unordered_set<std::string> triggers;
  for (size_t i = 0; i < getNumChildren(); i++) {
    for (const Component* component : getChildByIndex(i).getComponents()) {
      const std::string trigger = component->full_name() + "/" + kStatusEventChannel;
      triggers.insert(trigger);
    }
  }
  tickOnEvents(triggers);
}

size_t Behavior::getNumChildren() const {
  return children().getNumNodes();
}

alice::Node& Behavior::getChildByIndex(size_t index) const {
  return children().getNodeByIndex(index);
}

alice::Node* Behavior::findChildByName(const std::string& name) const {
  return children().findNodeByName(name);
}

alice::Node& Behavior::getChildByName(const std::string& name) const {
  return children().getNodeByName(name);
}

alice::Status Behavior::getChildStatus(alice::Node& node) const {
  return node.getStatus();
}

void Behavior::startChild(alice::Node& other) const {
  node()->app()->backend()->node_backend()->startNode(&other);
}

void Behavior::stopChild(alice::Node& other) const {
  node()->app()->backend()->node_backend()->stopNode(&other);
}

NodeGroup& Behavior::children() const {
  if (children_ == nullptr) {
    children_ = node()->getComponent<NodeGroup>();
  }
  return *children_;
}

}  // namespace behavior_tree
}  // namespace isaac
