/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

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
// A simple behavior which updates its status to match the parameter. It can be used for tests, to
// build more complicated behavior trees, or prototype applications where parts of the behavior
// tree is not implemented yet. If tick_period parameter is set, this codelet will tick periodically
// to check the status parameter.
class ConstantBehavior : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // The desired status of the behavior.
  ISAAC_PARAM(alice::Status, status, alice::Status::RUNNING);
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::ConstantBehavior);
