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
namespace opencv {

// This codelet computes the Sobel Gradient of an image using OpenCV
class EdgeDetector : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Input image and model parameters
  ISAAC_PROTO_RX(ColorCameraProto, input_image);
  // Output image and model parameters
  ISAAC_PROTO_TX(ColorCameraProto, output_image);
  // OpenCV kernel size for Sobel Operator, per OpenCV should be 1, 3, 5 or 7
  ISAAC_PARAM(int, kernel_size, 3);
};

}  // namespace opencv
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::opencv::EdgeDetector);
