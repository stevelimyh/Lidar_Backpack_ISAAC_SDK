/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "messages/camera.capnp.h"

namespace isaac {
namespace message_generators {

// Reads images from file systems and outputs them as messages. This can for example be used to
// create mock up tests when no camera hardware is available. This codelet encodes the raw image as
// a ColorCameraProto message containing the RGB image (and camera intrinsic information).
class ImageLoader : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output the color camera image proto
  ISAAC_PROTO_TX(ColorCameraProto, color);
  // Output the depth image proto
  ISAAC_PROTO_TX(DepthCameraProto, depth);

  // Path of the color image file. The image is expected to be a 3-channel RGB PNG.
  ISAAC_PARAM(std::string, color_filename);
  // Path of the depth image file. The image is expected to be a 1-channel 16-bit grayscale PNG.
  ISAAC_PARAM(std::string, depth_filename);
  // Path of the color image directory. The directory is expected to contain only 3-channel RGB PNG.
  // The directory name should be specified according to the rules set used by the shell (See
  // glob(7), POSIX.2, 3.13).
  // eg: './*' locates all file names in ./
  //     './*.py' locates all .py files in ./
  ISAAC_PARAM(std::string, color_glob_pattern);
  // The images in the specified directory plays in a loop if set to true. Otherwise it plays once.
  ISAAC_PARAM(bool, loop_images, true);
  // The images in the directory are sorted by NUMBER when they are 'NUMBER.jpg' or 'NUMBER.png'.
  ISAAC_PARAM(bool, sort_by_number, false);
  // A scale parameter to convert 16-bit depth to f32 depth
  ISAAC_PARAM(double, depth_scale, 0.001);
  // Image undistortion model. Must be 'brown' or 'fisheye'
  ISAAC_PARAM(std::string, distortion_model, "brown");
  // Focal length in pixels
  ISAAC_PARAM(Vector2d, focal_length);
  // Optical center in pixels
  ISAAC_PARAM(Vector2d, optical_center);
  // Distortion coefficients (see the DistortionProto in Camera.capnp for details)
  ISAAC_PARAM(Vector5d, distortion_coefficients, Vector5d::Zero());
  // Minimum depth
  ISAAC_PARAM(double, min_depth, 0.0);
  // Maximum depth
  ISAAC_PARAM(double, max_depth, 10.0);

 private:
  std::optional<Image3ub> color_image_;
  std::optional<Image1f> depth_image_;
  std::vector<Image3ub> color_images_;
  size_t img_idx_ = 0;

  // Input: pointer to the image to be published and the image's acquired time
  // Output: Sets pinhole and distortion and publishes the image
  // Assumes .png images don't contain alpha channel, only rgb
  void createAndPublishColorImageMessage(Image3ub image, const int64_t acqtime);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::ImageLoader);
