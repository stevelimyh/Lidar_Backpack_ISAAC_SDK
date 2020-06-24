/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

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

// Abstract interface (component) that helps set up rewards for reinforcement learning.
// StateMachineGymFlow expects one clone of this component for each agent in simulation.
// Rewards are gained by the agent if the action that was performed helped the agent
// get closer to the center of the target dolly.
class Reward: public alice::Component {
 public:
  // Function to initialize component variables for Reward
  virtual void init() {}
  // The evaluate function takes in a tuple of the form (s,a,s') : (new state, action, old state)
  // along with a bool signifying if an agent is dead or alive and some optional auxillary
  // information. It returns a float representing the reward obtained for the transition.
  // The expectation is that the robot started in state (referred to as the old state), takes an
  // action and reaches a new state. For this transition, it receives a reward calculated by this
  // module.
  virtual float evaluate(TensorConstView1f new_state_vector,
                         TensorConstView1f action_vector,
                         TensorConstView1f old_state_vector,
                         bool dead,
                         TensorView1f aux) = 0;
};

}  // namespace rl
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::rl::Reward);
