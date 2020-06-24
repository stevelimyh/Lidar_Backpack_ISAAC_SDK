/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "packages/behavior_tree/components/Behavior.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// A behavior which will execute all children one after another children. The behavior will succeed
// if all its children succeeded or if there are no children. In case a child failed the sequence
// behavior will fail.
// This behavior has a memory and remembers which child it is currently executing. This is a
// modification of the default sequence behavior.
class MemorySequenceBehavior : public Behavior {
 public:
  void start() override;
  void tick() override;

 private:
  // The index of the currently active child.
  size_t current_child_index_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::MemorySequenceBehavior);
