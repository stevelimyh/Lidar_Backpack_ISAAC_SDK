/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/camera.capnp.h"
#include "messages/tensor.capnp.h"

namespace isaac {
namespace message_generators {

// ImageFeatureGenerator generates an animated chessboard image with 2d features selected.
// The 2d features are the centers of the cells. On each tick, the image moves at
// right down corner.
class ImageFeatureGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output mockup image
  ISAAC_PROTO_TX(ColorCameraProto, image);
  // Output keypoint coordinates
  ISAAC_PROTO_TX(TensorProto, coordinates);
  // Output features ids
  ISAAC_PROTO_TX(TensorProto, features);

  // Chessboard cell size in pixel
  ISAAC_PARAM(int, cell_size, 50);
  // Image height in pixels
  ISAAC_PARAM(int, image_rows, 512);
  // Image width in pixels
  ISAAC_PARAM(int, image_cols, 512);
  // Chessboard shift per tick in pixels
  ISAAC_PARAM(float, animation_speed, 2.0);

 private:
  float shift_ = 0;  // animation state
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::ImageFeatureGenerator);
