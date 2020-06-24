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
#include "messages/point_cloud.capnp.h"

namespace isaac {
namespace message_generators {

// Generate point cloud messages to send out.
class PointCloudGenerator : public alice::Codelet {
 public:
  // Outgoing proto messages used to publish the point cloud messages.
  ISAAC_PROTO_TX(PointCloudProto, point_cloud);

  // Total number of point to generate.
  ISAAC_PARAM(int, point_count, 10000);
  // Maximum number of points in a single given message.
  ISAAC_PARAM(int, point_per_message, 100);
  // Whether there should be normals in the messages, as many as the number of points.
  ISAAC_PARAM(bool, has_normals, false);
  // Whether there should be colors in the messages, as many as the number of points.
  ISAAC_PARAM(bool, has_colors, false);
  // Whether there should be intensities in the messages, as many as the number of points.
  ISAAC_PARAM(bool, has_intensities, false);

  void start() override;
  void tick() override;

 private:
  // Send one message of points per remaining points.
  void sendBatch();

  // The number of remaining points to send, this value will decrease over time ticks.
  int remaining_points_;
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::PointCloudGenerator);
