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

namespace isaac {
namespace behavior_tree {

class NodeGroup;

// @experimental
// Base class for behaviors with children. Note that every node can act as a behavior, however this
// base class provides features for behaviors with children. Most notable it will automatically
// stop all children when the behavior stops. This component must be used in combination with a
// NodeGroup component.
class Behavior : public alice::Codelet {
 public:
  // The stop function will stop all children which are still running. Be careful to call this
  // function or take care to stop children yourself in case you override this method in a derived
  // class.
  void stop() override;

  // Ticks whenever a child changes its status
  void tickOnChildStatus();

  // The number of chilren
  size_t getNumChildren() const;
  // Gets the child at the given index. Will assert if the index is out of range.
  alice::Node& getChildByIndex(size_t index) const;
  // Gets the child with the given name. This function will assert in case there is no child node
  // with the given name.
  alice::Node& getChildByName(const std::string& name) const;
  // Looks for a child node with the given name and returns a pointer to it. This function returns
  // nullptr if there is no child with the given name.
  alice::Node* findChildByName(const std::string& name) const;

  // Gets the status of a child node. The node status is the combination of the stati of all its
  // components. If any component is in failure, the node is in failure. If all components are in
  // the success state the node is in success state.
  alice::Status getChildStatus(alice::Node& node) const;
  // Same as getChildStatus(getChildByIndex(index))
  alice::Status getChildStatus(size_t index) const {
    return getChildStatus(getChildByIndex(index));
  }
  // Same as getChildStatus(getChildByName(name))
  alice::Status getChildStatus(const std::string& name) const {
    return getChildStatus(getChildByName(name));
  }

  // Starts a child node.
  void startChild(alice::Node& node) const;
  // Same as startChild(getChildByIndex(index))
  void startChild(size_t index) const {
    startChild(getChildByIndex(index));
  }
  // Same as startChild(getChildByName(name))
  void startChild(const std::string& name) const {
    startChild(getChildByName(name));
  }

  // Stops a child node.
  void stopChild(alice::Node& node) const;
  // Same as stopChild(getChildByIndex(index))
  void stopChild(size_t index) const {
    stopChild(getChildByIndex(index));
  }
  // Same as stopChild(getChildByName(name))
  void stopChild(const std::string& name) const {
    stopChild(getChildByName(name));
  }

 private:
  // Gets the node group containing the children
  NodeGroup& children() const;

  // A link to the node group containing the children. Will be automatically computed by a call to
  // the children() function.
  mutable NodeGroup* children_ = nullptr;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::Behavior);
