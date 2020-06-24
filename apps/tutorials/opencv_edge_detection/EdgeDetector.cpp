/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "EdgeDetector.hpp"

#include <memory>
#include <utility>

#include "engine/core/assert.hpp"

#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"
#include "messages/math.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace isaac {
namespace opencv {

void EdgeDetector::start() {
  tickOnMessage(rx_input_image());
}

void EdgeDetector::tick() {
  const int kernel_size = get_kernel_size();
  switch (kernel_size) {
    case 1:
    case 3:
    case 5:
    case 7:
      break;
    default:
      LOG_ERROR("Invalid Kernel size for edge detector, Must be 1, 3, 5 or 7. Aborting.");
      return;
  }

  auto input = rx_input_image().getProto();
  ImageConstView3ub input_image;
  bool ok = FromProto(input.getImage(), rx_input_image().buffers(), input_image);
  ASSERT(ok, "Failed to deserialize the input image");

  const size_t rows = input_image.rows();
  const size_t cols = input_image.cols();
  Image1ub output_image(rows, cols);
  cv::Mat image =
      cv::Mat(rows, cols, CV_8UC3,
              const_cast<void*>(static_cast<const void*>(input_image.data().pointer())));
  cv::Mat gradient = cv::Mat(rows, cols, CV_8U, static_cast<void*>(output_image.data().pointer()));

  cv::Mat scratch, scratch_gray_scale;
  cv::GaussianBlur(image, scratch, cv::Size(kernel_size, kernel_size), 0, 0, cv::BORDER_DEFAULT);
  cv::cvtColor(scratch, scratch_gray_scale, cv::COLOR_RGB2GRAY);

  cv::Mat gradient_x, gradient_y;
  cv::Mat abs_gradient_x, abs_gradient_y;
  cv::Sobel(scratch_gray_scale, gradient_x, CV_16S, 1, 0, kernel_size);
  cv::Sobel(scratch_gray_scale, gradient_y, CV_16S, 0, 1, kernel_size);
  cv::convertScaleAbs(gradient_x, abs_gradient_x);
  cv::convertScaleAbs(gradient_y, abs_gradient_y);
  cv::addWeighted(abs_gradient_x, 0.5, abs_gradient_y, 0.5, 0, gradient);

  auto output = tx_output_image().initProto();

  auto in_pinhole = input.getPinhole();
  const Vector2d focal = FromProto(in_pinhole.getFocal());
  const Vector2d center = FromProto(in_pinhole.getCenter());

  auto out_pinhole = output.initPinhole();
  ToProto(focal, out_pinhole.getFocal());
  ToProto(center, out_pinhole.getCenter());
  out_pinhole.setCols(in_pinhole.getCols());
  out_pinhole.setRows(in_pinhole.getRows());

  if (input.hasDistortion()) {
    auto in_distortion = input.getDistortion();
    auto out_distortion = output.initDistortion();
    auto distortion_coeff = FromProto(in_distortion.getCoefficients());
    out_distortion.setModel(in_distortion.getModel());
    ToProto(distortion_coeff, out_distortion.getCoefficients());
  }

  output.setColorSpace(ColorCameraProto::ColorSpace::GRAYSCALE);
  ToProto(std::move(output_image), output.getImage(), tx_output_image().buffers());
  tx_output_image().publish(rx_input_image().acqtime());
}

}  // namespace opencv
}  // namespace isaac
