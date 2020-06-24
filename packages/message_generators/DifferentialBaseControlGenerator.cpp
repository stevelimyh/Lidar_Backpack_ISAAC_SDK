/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "DifferentialBaseControlGenerator.hpp"

#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"

namespace isaac {
namespace message_generators {

void DifferentialBaseControlGenerator::start() {
  tickPeriodically();
}

void DifferentialBaseControlGenerator::tick() {
  messages::DifferentialBaseControl control;
  control.linear_speed() = get_linear_speed();
  control.angular_speed() = get_angular_speed();
  ToProto(control, tx_command().initProto(), tx_command().buffers());
  tx_command().publish();
}

}  // namespace message_generators
}  // namespace isaac
