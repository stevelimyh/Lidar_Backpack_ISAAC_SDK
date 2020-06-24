/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "PointCloudGenerator.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/tensor/sample_cloud.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

void PointCloudGenerator::start() {
  // Prime the number of points remaining to be sent per the parameter.
  remaining_points_ = get_point_count();

  tickPeriodically();
}

void PointCloudGenerator::tick() {
  // Send a batch per tick, as long as we have remaining points to send.
  sendBatch();
}

void PointCloudGenerator::sendBatch() {
  // Handle reamining count and ensure we don't send more than we should.
  if (remaining_points_ > 0) {
    const int points_to_send = std::min(remaining_points_, get_point_per_message());
    const bool has_normals = get_has_normals();
    const bool has_colors = get_has_colors();
    const bool has_intensities = get_has_intensities();

    // Create buffers to hold the point cloud data
    SampleCloud3f positions(points_to_send);
    SampleCloud3f normals;
    SampleCloud3f colors;
    SampleCloud1f intensities;
    if (has_normals) normals.resize(points_to_send);
    if (has_colors) colors.resize(points_to_send);
    if (has_intensities) intensities.resize(points_to_send);

    // Initialize and populate the message per test requirements.
    // Generate a wavy surface as test data.
    for (int point_index = 0; point_index < points_to_send; point_index++) {
      const float u = static_cast<float>(point_index % 100);
      const float v = static_cast<float>(point_index / 100);
      const float phase = u / 30.0f;
      const float dx = std::cos(phase);
      positions(point_index) = Vector3f{u, v, 10.0f * dx};
      if (has_normals) {
        normals(point_index) = positions(point_index).normalized();
      }
      if (has_colors) {
        const float tint = dx * 0.5f + 0.5f;
        colors(point_index) = Vector3f{tint, tint, tint};
      }
      if (has_intensities) {
        intensities(point_index) = std::sin(phase) * 0.5f + 0.5f;
      }
    }

    // Write the proto
    auto proto = tx_point_cloud().initProto();
    std::vector<SharedBuffer>& buffers = tx_point_cloud().buffers();
    ToProto(std::move(positions), proto.initPositions(), buffers);
    if (has_normals) {
      ToProto(std::move(normals), proto.initNormals(), buffers);
    }
    if (has_colors) {
      ToProto(std::move(colors), proto.initColors(), buffers);
    }
    if (has_intensities) {
      ToProto(std::move(intensities), proto.initIntensities(), buffers);
    }

    // Publish
    tx_point_cloud().publish();
    remaining_points_ -= points_to_send;
  }
  // Else: We do nothing if there is no more data to send.
}

}  // namespace message_generators
}  // namespace isaac
