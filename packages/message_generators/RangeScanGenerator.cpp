/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RangeScanGenerator.hpp"

#include <algorithm>
#include <utility>

#include "engine/gems/interpolation/utils.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

namespace {
// Ray intensity. 0xFF is the maximum.
const uint8_t kIntensity = 255;
}  // namespace

void RangeScanGenerator::start() {
  slice_index_ = 0;
  tickPeriodically();
}

void RangeScanGenerator::tick() {
  // Read and check parameters
  const double max_range = get_max_range();
  if (max_range <= 0.0) {
    reportFailure("Max range needs to be positive.");
    return;
  }
  const float range_domain_max = get_range_domain_max();
  if (range_domain_max <= max_range) {
    reportFailure("Range domain max needs to be larger than max range");
    return;
  }
  const double min_range = get_min_range();
  if (min_range < 0.0) {
    reportFailure("Min range needs to be non-negative.");
    return;
  }
  const int delta_time = get_delta_time();
  if (delta_time <= 0) {
    reportFailure("Delta time needs to be positive.");
    return;
  }
  const auto vertical_beam_angles = get_vertical_beam_angles();
  if (vertical_beam_angles.empty()) {
    reportFailure("Beam vertical angles should not be empty.");
    return;
  }
  const int num_slices_pre = get_num_slices();
  if (num_slices_pre <= 0) {
    reportFailure("Number of slices needs to be positive.");
    return;
  }
  const size_t num_slices = static_cast<size_t>(num_slices_pre);
  int num_slices_per_message_pre = get_num_slices_per_message();
  if (num_slices_per_message_pre == 0) {
    // Special case
    num_slices_per_message_pre = num_slices;
  } else if (num_slices_per_message_pre < 0) {
    reportFailure("Number of slices needs to be non-negative.");
    return;
  } else if (num_slices_per_message_pre > num_slices_pre) {
    reportFailure(
        "Number of slices per message needs to be less than or equal to the number of slices in "
        "total.");
    return;
  }
  const size_t num_slices_per_message = static_cast<size_t>(num_slices_per_message_pre);
  const double intensity_denormalizer = get_intensity_denormalizer();
  const Vector2f azimuth_angle_range = get_azimuth_angle_range().cast<float>();

  // Create outgoing message
  auto proto = tx_scan().initProto();
  proto.setRangeDenormalizer(range_domain_max);
  proto.setIntensityDenormalizer(intensity_denormalizer);
  proto.setDeltaTime(delta_time);
  proto.setInvalidRangeThreshold(min_range);
  proto.setOutOfRangeThreshold(max_range);

  auto phi_proto = proto.initPhi(vertical_beam_angles.size());
  for (size_t j = 0; j < vertical_beam_angles.size(); j++) {
    phi_proto.set(j, static_cast<float>(vertical_beam_angles[j]));
  }

  auto thetas_proto = proto.initTheta(num_slices_per_message);
  Tensor2ui16 ranges(num_slices_per_message, vertical_beam_angles.size());
  Tensor2ub intensities(num_slices_per_message, vertical_beam_angles.size());
  for (size_t i = 0; i < num_slices_per_message; i++) {
    // Note that i may not equal slice_index_ if num_slices_per_message does not equal num_slices
    thetas_proto.set(i, RescaleFromInteger(slice_index_, num_slices, azimuth_angle_range[0],
                                           azimuth_angle_range[1]));
    for (size_t j = 0; j < vertical_beam_angles.size(); j++) {
      const float range_meters = castRay(vertical_beam_angles[j], max_range);
      const uint16_t range_normalized =
          RescaleToInteger(range_meters, 0.0f, range_domain_max, 0xffff);
      ranges(i, j) = range_normalized;
      intensities(i, j) = kIntensity;
    }
    // Increment the slice index. Wrap around.
    slice_index_++;
    if (slice_index_ == num_slices) {
      slice_index_ = 0;
    }
  }
  ToProto(std::move(ranges), proto.initRanges(), tx_scan().buffers());
  ToProto(std::move(intensities), proto.initIntensities(), tx_scan().buffers());

  tx_scan().publish();
}

double RangeScanGenerator::castRay(double phi, double max_range) const {
  const auto beam = geometry::Ray2d::FromDirection(Vector2d{0.0, get_height()},
                                                   Vector2d{std::cos(phi), std::sin(phi)});
  double range = max_range;
  for (const auto& segment : get_segments()) {
    double current_range;
    if (geometry::AreLinesIntersecting(beam, segment, &current_range)) {
      range = std::min(range, current_range);
    }
  }
  return range;
}

}  // namespace message_generators
}  // namespace isaac
