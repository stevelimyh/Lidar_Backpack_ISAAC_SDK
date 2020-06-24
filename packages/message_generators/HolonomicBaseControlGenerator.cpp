/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "HolonomicBaseControlGenerator.hpp"

#include "engine/gems/state/io.hpp"
#include "messages/state/holonomic_base.hpp"

namespace isaac {
namespace message_generators {

void HolonomicBaseControlGenerator::start() {
  tickPeriodically();
}

void HolonomicBaseControlGenerator::tick() {
  // Publish command with desired speed values
  messages::HolonomicBaseControls controls;
  controls.speed_x() = get_speed_linear_x();
  controls.speed_y() = get_speed_linear_y();
  controls.angular_speed() = get_speed_angular();
  ToProto(controls, tx_command().initProto(), tx_command().buffers());
  tx_command().publish();
}

}  // namespace message_generators
}  // namespace isaac
