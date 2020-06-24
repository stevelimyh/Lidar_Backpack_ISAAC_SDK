/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/state.capnp.h"

namespace isaac {
namespace message_generators {

// Generates periodic PanTiltState with specified pattern
class PanTiltStateGenerator : public alice::Codelet {
 public:
  // Available output modes for ImageWarp
  enum class WaveMode { kSinus, kTriangle, kInvalid = -1 };

  void start() override;
  void tick() override;

  // Output, List of AprilTag fiducials
  ISAAC_PROTO_TX(StateProto, target);

  // Max panning on one side in Rad
  ISAAC_PARAM(double, pan_max_angle, 1.0471975511965976);
  // Degree offset for panning
  ISAAC_PARAM(double, pan_offset_angle, 0.0);
  // Speed for panning in round/second
  ISAAC_PARAM(double, pan_speed, 0.1);
  // Wave function for panning
  ISAAC_PARAM(WaveMode, pan_mode, WaveMode::kSinus)

  // Max tilting on one side in degree
  ISAAC_PARAM(double, tilt_max_angle, 0.5235987755982988);
  // Degree offset of tilting
  ISAAC_PARAM(double, tilt_offset_angle, 0.0);
  // Speed for tilting in round/second
  ISAAC_PARAM(double, tilt_speed, 0.1);
  // Wave function for tilting
  ISAAC_PARAM(WaveMode, tilt_mode, WaveMode::kSinus)

 private:
  // Calculates a wave function as specified by mode for time t with T=1
  double calculateAngle(const double time, const double speed, const double max_angle,
                        const WaveMode mode, const double offset_angle);
};

NLOHMANN_JSON_SERIALIZE_ENUM(PanTiltStateGenerator::WaveMode,
                             {{PanTiltStateGenerator::WaveMode::kInvalid, nullptr},
                              {PanTiltStateGenerator::WaveMode::kSinus, "sinus"},
                              {PanTiltStateGenerator::WaveMode::kTriangle, "triangle"}});

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::PanTiltStateGenerator);
