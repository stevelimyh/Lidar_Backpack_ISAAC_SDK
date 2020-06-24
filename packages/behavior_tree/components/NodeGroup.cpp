/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "NodeGroup.hpp"

#include <string>

namespace isaac {
namespace behavior_tree {

void NodeGroup::start() {
  for (const std::string& node_name : get_node_names()) {
    alice::Node* child = node()->app()->findNodeByName(node_name);
    if (child == nullptr) {
      reportFailure("could not find child");
      return;
    }
    nodes_.push_back(child);
    nodes_by_name_[node_name] = child;
  }
  reportSuccess();
  can_access_children_ = true;
}

void NodeGroup::stop() {
  can_access_children_ = false;
  nodes_.clear();
  nodes_by_name_.clear();
}

size_t NodeGroup::getNumNodes() const {
  assertChildAccess();
  return nodes_.size();
}

alice::Node& NodeGroup::getNodeByIndex(size_t index) {
  assertChildAccess();
  ASSERT(index < nodes_.size(), "Index out of range: %zd >= %zd", index, nodes_.size());
  return *nodes_[index];
}

alice::Node& NodeGroup::getNodeByName(const std::string& name) {
  assertChildAccess();
  alice::Node* node = findNodeByName(name);
  ASSERT(node != nullptr, "No node with name '%s'", name.c_str());
  return *node;
}

alice::Node* NodeGroup::findNodeByName(const std::string& name) {
  assertChildAccess();
  const auto it = nodes_by_name_.find(name);
  return it != nodes_by_name_.end() ? it->second : nullptr;
}

void NodeGroup::assertChildAccess() const {
  ASSERT(can_access_children_, "Node group can only be accessed during the running phase, not "
         "before start or after stop");
}

}  // namespace behavior_tree
}  // namespace isaac
