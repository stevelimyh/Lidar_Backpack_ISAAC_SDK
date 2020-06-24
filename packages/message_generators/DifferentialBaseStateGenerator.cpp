/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "DifferentialBaseStateGenerator.hpp"

#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"

namespace isaac {
namespace message_generators {

void DifferentialBaseStateGenerator::start() {
  tickPeriodically();
}

void DifferentialBaseStateGenerator::tick() {
  messages::DifferentialBaseDynamics diff_base_state;
  diff_base_state.linear_speed() = get_linear_speed();
  diff_base_state.angular_speed() = get_angular_speed();
  diff_base_state.linear_acceleration() = get_linear_acceleration();
  diff_base_state.angular_acceleration() = get_angular_acceleration();
  ToProto(diff_base_state, tx_state().initProto(), tx_state().buffers());
  tx_state().publish();
}

}  // namespace message_generators
}  // namespace isaac
