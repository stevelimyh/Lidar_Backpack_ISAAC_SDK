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

// Abstract interface (component) that helps decide if an agent needs to be killed
// for being in an invalid state.
// StateMachineGymFlow expects one clone of this codelet for each agent in simulation
// and evaluates the state of a single agent once every loop of the state machine.
class Death: public alice::Component {
 public:
  // Function to initialize component variables for Death
  virtual void init() {}

  // The evaluate function returns true if the agent is in an invalid state
  // or has performed an invalid action
  virtual bool evaluate(TensorConstView1f state, TensorConstView1f action, TensorView1f aux) = 0;
};

}  // namespace rl
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::rl::Death);
