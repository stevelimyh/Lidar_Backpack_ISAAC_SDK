/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CameraGenerator.hpp"

#include <utility>

#include "messages/image.hpp"

namespace isaac {
namespace message_generators {

void CameraGenerator::start() {
  tickPeriodically();
}

void CameraGenerator::tick() {
  const int rows = get_rows();
  const int cols = get_cols();

  Image3ub buffer_color_left(rows, cols);
  Image3ub buffer_color_right(rows, cols);
  Image1f buffer_depth(rows, cols);

  const int64_t acqtime = getTickTimestamp();
  const int64_t offset = acqtime/2000000;
  // Create dynamic data
  for (int i = 0; i < buffer_color_left.rows(); i++) {
    const int64_t ii = static_cast<int64_t>(i) + offset;
    for (int j = 0; j < buffer_color_left.cols(); j++) {
      const int64_t jjp = static_cast<int64_t>(j) + offset;
      const int64_t jjm = static_cast<int64_t>(j) - offset;
      const unsigned char ci = static_cast<unsigned char>(ii);
      const unsigned char cjp = static_cast<unsigned char>(jjp);
      const unsigned char cjm = static_cast<unsigned char>(jjm);
      const unsigned char cx = static_cast<unsigned char>(i * j);
      buffer_color_left(i, j) = Pixel3ub{ci, cjp, cx};
      buffer_color_right(i, j) = Pixel3ub{ci, cjm, cx};

      const float q
          = 2.0f*static_cast<float>(j) / static_cast<float>(buffer_color_left.cols()) - 1.0f;
      buffer_depth(i, j) = 1.5f + 5.f*q*q
          + 0.5f*std::cos(0.05f*static_cast<float>(ii))
          + 0.5f*std::cos(0.05f*static_cast<float>(jjp));
    }
  }

  // TODO publish pinhole and distortion models

  // left color image
  auto proto_color_left = tx_color_left().initProto();
  ToProto(std::move(buffer_color_left), proto_color_left.initImage(),
          tx_color_left().buffers());
  proto_color_left.setColorSpace(ColorCameraProto::ColorSpace::RGB);
  tx_color_left().publish(acqtime);
  // right color image
  auto proto_color_right = tx_color_right().initProto();
  ToProto(std::move(buffer_color_right), proto_color_right.initImage(),
          tx_color_right().buffers());
  proto_color_right.setColorSpace(ColorCameraProto::ColorSpace::RGB);
  tx_color_right().publish(acqtime);
  // left color image
  auto proto_depth = tx_depth().initProto();
  ToProto(std::move(buffer_depth), proto_depth.initDepthImage(),
          tx_depth().buffers());
  proto_depth.setMinDepth(0.1f);
  proto_depth.setMaxDepth(10.0f);
  tx_depth().publish(acqtime);
}

void CameraGenerator::stop() {}

}  // namespace message_generators
}  // namespace isaac
