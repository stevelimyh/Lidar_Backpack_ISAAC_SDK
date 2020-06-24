/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "FlatscanGenerator.hpp"

#include "engine/alice/components/Random.hpp"
#include "engine/gems/interpolation/utils.hpp"

namespace isaac {
namespace message_generators {

void FlatscanGenerator::start() {
  tickPeriodically();
}

void FlatscanGenerator::tick() {
  // Read parameters
  const float invalid_range_threshold = get_invalid_range_threshold();
  const float out_of_range_threshold = get_out_of_range_threshold();
  const size_t beam_count = get_beam_count();
  const Vector2f angles_range = get_angles_range().cast<float>();
  const float range_mean = get_range_mean();
  const std::optional<float> maybe_range_standard_deviation = try_get_range_standard_deviation();

  // Get and check random component
  alice::Random* random = node()->getComponentOrNull<alice::Random>();
  if (maybe_range_standard_deviation) {
    if (random == nullptr) {
      reportFailure(
          "range_standard_deviation is set, but there is not alice::Random component in this node");
      return;
    }
  }

  // Create and publish message
  auto flatscan_proto = tx_flatscan().initProto();
  flatscan_proto.setInvalidRangeThreshold(invalid_range_threshold);
  flatscan_proto.setOutOfRangeThreshold(out_of_range_threshold);
  auto flatscan_angles_proto = flatscan_proto.initAngles(beam_count);
  auto flatscan_ranges_proto = flatscan_proto.initRanges(beam_count);
  for (size_t i = 0; i < beam_count; i++) {
    float range = range_mean;
    if (maybe_range_standard_deviation) {
      ASSERT(random, "Logic error. We should have reported failure above.");
      range += random->sampleGaussian(*maybe_range_standard_deviation);
    }
    flatscan_ranges_proto.set(i, range);
    flatscan_angles_proto.set(i,
                              RescaleFromInteger(i, beam_count, angles_range[0], angles_range[1]));
  }
  tx_flatscan().publish();
}

}  // namespace message_generators
}  // namespace isaac
