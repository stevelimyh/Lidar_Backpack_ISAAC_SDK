/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/alice/component.hpp"

namespace isaac {
namespace rl {

// Abstract interface (component) that plugs into StateMachineGymFlow to help
// respawn killed agents in simulation.
// StateMachineGymFlow expects one clone of this component for each agent in simulation,
// and this component is responsible for publishing teleport messages to simulation when
// the state machine calls its spawn function. The spawn function is only called when
// the agent and its scene need to be reset.
// This codelet is also responsible for randomizing the various objects
// in the scene. The output channel of this codelet must be connected to the Teleport
// codelet for the agent scene in simulation.
class Birth: public alice::Component {
 public:
  // Function to initialize component variables for Birth
  virtual void init() {}

  // The spawn function takes in the agent index signifying the position of the agent in the
  // aggregate tensor. This component is responsible for computing the new poses for
  // agents in the scene and sending appropriate reset messages to simulation
  virtual void spawn(int agent_index) = 0;
};

}  // namespace rl
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::rl::Birth);
