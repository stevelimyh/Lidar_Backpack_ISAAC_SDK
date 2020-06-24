/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// A list of nodes. This is for example used for behaviors with children.
class NodeGroup : public alice::Codelet {
 public:
  void start() override;
  void stop() override;

  // A list of node names. This is only read when the component starts.
  ISAAC_PARAM(std::vector<std::string>, node_names, {});

  // The number of nodes in this group
  size_t getNumNodes() const;
  // Gets a node in the group by index. If the index is invalid this function will assert.
  alice::Node& getNodeByIndex(size_t index);
  // Gets a node in this group by name. If there is no node with the given name this function will
  // assert. Use findNodeByName if you are not sure that a node with the name exists.
  alice::Node& getNodeByName(const std::string& name);
  // Finds a node in this group by name. If there is no node with the given name this function will
  // return a nullptr.
  alice::Node* findNodeByName(const std::string& name);

 private:
  // Asserts if the codelet is not running
  void assertChildAccess() const;

  // A flag to check if child access is allowed
  std::atomic<bool> can_access_children_;
  // The nodes in this group in sequences
  std::vector<alice::Node*> nodes_;
  // The nodes in this group accessibly by name. This is used for faster by-name lookups.
  std::unordered_map<std::string, alice::Node*> nodes_by_name_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::NodeGroup);
