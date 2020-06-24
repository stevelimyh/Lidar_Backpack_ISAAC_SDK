/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/flatscan.capnp.h"

namespace isaac {
namespace message_generators {

// FlatscanGenerator publishes a FlatscanProto periodically.
// Angles are parameterized. Ranges can be optionally randomized by adding an alice::Random
// component to the same node and setting a range_standard_deviation.
class FlatscanGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Outgoing "flat" range scan
  ISAAC_PROTO_TX(FlatscanProto, flatscan);

  // Beams with a range smaller than or equal to this distance are considered to have returned an
  // invalid measurement.
  ISAAC_PARAM(double, invalid_range_threshold, 0.2);
  // Beams with a range larger than or equal to this distance are considered to not have hit an
  // obstacle within the maximum possible range of the sensor.
  ISAAC_PARAM(double, out_of_range_threshold, 100.0);
  // Number of beams in outgoing message
  ISAAC_PARAM(int, beam_count, 1800);
  // Azimuth angle range for the beams
  ISAAC_PARAM(Vector2d, angles_range, Vector2d(0.0, TwoPi<double>));
  // Mean value for the ranges.
  ISAAC_PARAM(double, range_mean, 20.0);
  // Standard deviation for the range values. Requires an alice::Random component in the same node.
  ISAAC_PARAM(double, range_standard_deviation);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::FlatscanGenerator);
