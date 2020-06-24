/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/message_generators/ImageFeatureGenerator.hpp"

#include <utility>

#include "messages/camera.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace message_generators {

void ImageFeatureGenerator::start() {
  tickPeriodically();
}

void ImageFeatureGenerator::tick() {
  const int cols = get_image_cols();
  const int rows = get_image_rows();
  const int cell_sizes = get_cell_size();

  const int cells_row = rows / cell_sizes;
  const int cells_col = cols / cell_sizes;
  const int num_features = cells_row * cells_col;

  // generate coordinates
  Tensor2f coordinates(num_features, 2);
  const int64_t timestmamp = getTickTimestamp();
  for (int i = 0; i < num_features; i++) {
    coordinates(i, 0) = (shift_ + cell_sizes / 2.f + (i % cells_row) * cell_sizes);
    coordinates(i, 1) = (shift_ + cell_sizes / 2.f + (i / cells_row) * cell_sizes);
  }

  ToProto(std::move(coordinates), tx_coordinates().initProto(), tx_coordinates().buffers());

  // generate features
  Tensor1d features(num_features);
  for (int i = 0; i < num_features; i++) {
    features(i) = i;
  }
  ToProto(std::move(features), tx_features().initProto(), tx_features().buffers());

  // generate image
  auto camera = tx_image().initProto();
  camera.setColorSpace(ColorCameraProto::ColorSpace::GRAYSCALE);

  Image1ub image(rows, cols);
  const int s = static_cast<int>(shift_);
  const int w = cell_sizes;
  for (int row = 0; row < image.rows(); row++) {
    for (int col = 0; col < image.cols(); col++) {
      image(row, col) = 255 * (((row - s) % (2 * w) < w) ^ ((col - s) % (2 * w) < w));
    }
  }
  ToProto(std::move(image), camera.getImage(), tx_image().buffers());
  tx_features().publish(timestmamp);
  tx_coordinates().publish(timestmamp);
  tx_image().publish(timestmamp);

  // animate
  shift_ = shift_ < cell_sizes ? shift_ + get_animation_speed() : 0.f;
}

}  // namespace message_generators
}  // namespace isaac
