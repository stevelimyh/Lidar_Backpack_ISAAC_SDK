/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/status.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// A simple behavior which starts in the running state and after a given time period switches to
// either success or failure.
class TimerBehavior : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // The time delay after which the behavior switches to the desired status.
  ISAAC_PARAM(double, delay, 1.0);
  // The status to which the behavior switches after the time delay. Must be SUCCESS or FAILURE.
  ISAAC_PARAM(alice::Status, status, alice::Status::SUCCESS);
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::TimerBehavior);
