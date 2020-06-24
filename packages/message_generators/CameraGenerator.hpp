/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "messages/camera.capnp.h"

namespace isaac {
namespace message_generators {

// CameraGenerator publishes left and right color images and a left depth image with made up data.
class CameraGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // Random left color image
  ISAAC_PROTO_TX(ColorCameraProto, color_left)
  // Random right color image
  ISAAC_PROTO_TX(ColorCameraProto, color_right)
  // Random depth image
  ISAAC_PROTO_TX(DepthCameraProto, depth)

  // The number of rows for generated data
  ISAAC_PARAM(int, rows, 1080);
  // The number of columns for generated data
  ISAAC_PARAM(int, cols, 1920);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::CameraGenerator);
