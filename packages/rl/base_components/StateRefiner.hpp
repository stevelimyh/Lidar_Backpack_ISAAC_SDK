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
#include "engine/core/tensor/tensor.hpp"

namespace isaac {
namespace rl {

// Abstract interface (component) that helps post-processing the state of the agent.
// StateMachineGymFlow expects one clone of this component for each agent in the simulation.
// This component is entirely optional, but if it is present, the process() function is called
// by Gym between sending the teleport message to simulation (through Birth) and computing
// the reward through the Reward Component.
class StateRefiner: public alice::Component {
 public:
  // Function to initialize component states for StateRefiner
  virtual void init() {}

  // Refines or post-processes the state of the agent based on the death flag (true if agent dies),
  // the latest actions sent from the neural network and the auxiliary tensor
  virtual void process(TensorView1f state, TensorConstView1f action, bool dead, TensorView1f aux) {}
};

}  // namespace rl
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::rl::StateRefiner);
