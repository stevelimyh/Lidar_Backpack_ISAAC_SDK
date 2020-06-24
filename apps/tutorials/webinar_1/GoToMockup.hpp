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

#include "engine/alice/alice_codelet.hpp"
#include "messages/differential_base.capnp.h"

namespace isaac {
namespace tutorials {

// Imitates GoTo class and navigation subgraph by receiving a goal message and providing a dummy
// feedback in return. Feedback is "not arrived" initially, which later changes to "arrived" over
// time. GoToMockup can be used to test goal-generating codelets and it can be later replaced with
// the actual GoTo codelet or navigation subgraph since its input (Goal2Proto) and output
// (Goal2FeedbackProto) match the inputs and outputs of GoTo codelet and navigation subgraph.
class GoToMockup : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Feedback about the last received goal
  ISAAC_PROTO_TX(Goal2FeedbackProto, feedback);

  // The target destination received
  ISAAC_PROTO_RX(Goal2Proto, goal);

  // Feedback will switch from "not arrived" to "arrived" after this many seconds.
  ISAAC_PARAM(double, time_until_arrival, 5.0);

 private:
  // Information about the desired goal
  struct Goal {
    // Acquisition time to which the goal relates
    int64_t acqtime;
    // The name of the goal coordinate frame
    std::string frame_name;
    // The pose of the goal in the goal coordinate frame
    Pose2d frame_T_goal;
  };

  // Reads input messages, updates goal timestamp, and starts timer for arrival.
  void processGoal();
  // Publishes feedback for the last goal received.
  void publishFeedback();

  // Save the last goal for two reasons:
  // 1. Goal acqtime is saved to publish feedback with correct timestamp,
  // 2. Pose and frame are saved to detect changes in goal location.
  std::optional<Goal> maybe_goal_;
};

}  // namespace tutorials
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::tutorials::GoToMockup);
