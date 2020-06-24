/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/range_scan.capnp.h"

namespace isaac {
namespace message_generators {

// RangeScanGenerator publishes a RangeScanProto periodically.
// Simulates a range scan in a radial-symmetric world with desired parameters.
class RangeScanGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Outgoing range scan
  ISAAC_PROTO_TX(RangeScanProto, scan);

  // Azimuth angle range for the beams. (2pi, 0) would produce counter-clockwise rotation.
  ISAAC_PARAM(Vector2d, azimuth_angle_range, Vector2d(0.0, TwoPi<double>));
  // Number of (horizontal) ray slices that cover azimuth_angle_range
  ISAAC_PARAM(int, num_slices, 16);
  // Number of (horizontal) ray slices published with each message. 0 means publish num_slices each
  // message. Needs to be smaller than num_slices.
  ISAAC_PARAM(int, num_slices_per_message, 0);
  // The (vertical) beam angles to use for every slice
  ISAAC_PARAM(std::vector<double>, vertical_beam_angles,
              std::vector<double>({DegToRad(-15.0), DegToRad(-7.0), DegToRad(-3.0), DegToRad(-1.0),
                                   DegToRad(+1.0), DegToRad(+3.0), DegToRad(+7.0),
                                   DegToRad(+15.0)}));
  // Out of range threshold
  ISAAC_PARAM(double, max_range, 100.0);
  // Invalid range threshold
  ISAAC_PARAM(double, min_range, 0.0);
  // Max value of the range domain. Used when normalizing range values.
  ISAAC_PARAM(double, range_domain_max, 110.0);
  // Delay in microseconds between firing. Default is 20 Hz.
  ISAAC_PARAM(int, delta_time, 50'000);
  // Scale factor which can be used to convert an intensity value from an 8-bit integer to meters.
  ISAAC_PARAM(double, intensity_denormalizer, 1.0);
  // The height of the lidar over the ground plane
  ISAAC_PARAM(double, height, 1.0);

  // Lines in range / height plane which define the world
  // Layout:
  // [
  //   [ [-100.0, 0.0], [100.0, 0.0] ],
  //   [ [   0.0, 0.0], [ 20.0, 2.0] ]
  // ]
  ISAAC_PARAM(std::vector<geometry::LineSegment2d>, segments, {});

 private:
  // Casts a ray with given angle against all segments and returns the range where it hits an
  // obstacle; or max_range if it did not hit an obstacle.
  double castRay(double phi, double max_range) const;

  // Index of the slice to publish next
  size_t slice_index_;
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::RangeScanGenerator);
