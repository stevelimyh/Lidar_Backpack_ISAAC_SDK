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
// A behavior which will execute it's one and only child. Once the child stopped successfuly, it
// will pause for a while and then start it again.
class RepeatBehavior : public Behavior {
 public:
  void start() override;
  void tick() override;

  // If this flag is enabled the child will also be repeated after it stopped with a failure.
  ISAAC_PARAM(bool, repeat_after_failure, false);

  // After the child stopped the repeat behavior waits for this time period before restarting the
  // child.
  ISAAC_PARAM(double, wait_duration, 1.0);

 private:
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::RepeatBehavior);
