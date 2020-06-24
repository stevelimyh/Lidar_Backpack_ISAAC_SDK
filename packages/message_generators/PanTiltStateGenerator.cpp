/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PanTiltStateGenerator.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "engine/gems/state/io.hpp"
#include "engine/gems/state/state.hpp"
#include "messages/state/pan_tilt.hpp"

namespace isaac {
namespace message_generators {

namespace {
// Calculates a triangle wave in [-1, 1] with T of 1
double TriangleWave(const double x) {
  double integer_part = 0;
  return 1.0f - std::abs(0.5f - std::modf(x, &integer_part)) * 4.0f;
}

}  // namespace

void PanTiltStateGenerator::start() {
  tickPeriodically();
}

void PanTiltStateGenerator::tick() {
  double pan = 0.0f;
  double tilt = 0.0f;

  pan = calculateAngle(getTickTime(), get_pan_speed(), get_pan_max_angle(), get_pan_mode(),
                       get_pan_offset_angle());

  tilt = calculateAngle(getTickTime(), get_tilt_speed(), get_tilt_max_angle(), get_tilt_mode(),
                        get_tilt_offset_angle());

  messages::PanTiltState state;
  state.pan() = pan;
  state.tilt() = tilt;

  ToProto(state, tx_target().initProto(), tx_target().buffers());
  tx_target().publish();
}

double PanTiltStateGenerator::calculateAngle(const double time, const double speed,
                                             const double max_angle, const WaveMode mode,
                                             const double offset_angle) {
  const double base = time * speed;
  switch (mode) {
    case WaveMode::kSinus:
      return max_angle * std::sin(base * M_PI * 2) + offset_angle;
      break;
    case WaveMode::kTriangle:
      return max_angle * TriangleWave(base) + offset_angle;
      break;
    default:
      reportFailure("Unsupported wave mode");
      return 0.0f;
  }
}

}  // namespace message_generators
}  // namespace isaac
