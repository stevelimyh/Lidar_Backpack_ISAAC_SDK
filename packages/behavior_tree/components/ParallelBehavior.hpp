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
// A behavior which executes its children in parallel at the same time. Its succeeds/fails if the
// number of succeeding/failing children exeeds given thresholds.
class ParallelBehavior : public Behavior {
 public:
  void start() override;
  void tick() override;

  // If the number of succeeding children is larger than or equal to this threshold, the parallel
  // behavior will succeed. If this is set to -1, it means that all children must succeed. Value
  // must be -1 or larger. By default all children must succeed.
  ISAAC_PARAM(int, success_threshold, -1);
  // Similar to `success_threshold`. By default the parallel will fail if any child fails. In case
  // that both success and failure conditions are satisfied at the same time, success takes
  // precedence.
  ISAAC_PARAM(int, failure_threshold, 1);
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::ParallelBehavior);
