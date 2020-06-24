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
#include "messages/differential_base.capnp.h"

namespace isaac {
namespace tutorials {

// Skeleton for tutorial in the first ISAAC webinar.
// For the complete version, please check GoalGenerator.hpp.
class MyGoalGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  ISAAC_PROTO_TX(Goal2Proto, my_goal);

  ISAAC_PROTO_RX(Goal2FeedbackProto, feedback_goal);

  ISAAC_PARAM(Vector2d, desired_location, Vector2d(2.0,4.0));
};

}  // namespace tutorials
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::tutorials::MyGoalGenerator);
