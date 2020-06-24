/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ImageLoader.hpp"

#include <glob.h>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"

namespace isaac {
namespace message_generators {

namespace {

// Input:  A directory pattern.
// Output: A vector containing all the file names that match the pattern.
std::vector<std::string> GlobFiles(const std::string& pattern, bool sort_by_number) {
  glob_t glob_result;
  glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
  std::vector<std::string> files;
  for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
    files.push_back(std::string(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
  // Sort the images by their integer number as opposed to the entire image string
  if (sort_by_number) {
    std::sort(files.begin(), files.end(), [](std::string first, std::string second){
      size_t first_start = first.rfind("/") + 1;
      size_t second_start = second.rfind("/") + 1;
      first = first.substr(first_start, first.size() - 4 - first_start);
      second = second.substr(second_start, second.size() - 4 - second_start);
      return (std::stoi(first) < std::stoi(second));
    });
  }
  return files;
}

}  // namespace

void ImageLoader::createAndPublishColorImageMessage(Image3ub image, const int64_t acqtime) {
  auto rgb_proto = tx_color().initProto();

  // Pinhole for color camera
  auto pinhole = rgb_proto.initPinhole();
  ToProto(get_focal_length(), pinhole.getFocal());
  ToProto(get_optical_center(), pinhole.getCenter());
  pinhole.setCols(image.cols());
  pinhole.setRows(image.rows());

  // Image for color camera
  ToProto(std::move(image), rgb_proto.initImage(), tx_color().buffers());
  rgb_proto.setColorSpace(ColorCameraProto::ColorSpace::RGB);

  // Distortion parameters for color camera
  auto distortion = rgb_proto.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  ToProto(get_distortion_coefficients(), distortion.getCoefficients());

  tx_color().publish(acqtime);
}

void ImageLoader::start() {
  bool tried_to_load_image = false;

  // Load color image
  color_image_ = std::nullopt;
  if (const auto maybe_filename = try_get_color_filename()) {
    tried_to_load_image = true;
    Image3ub color_image;
    if (LoadImage(*maybe_filename, color_image)) {
      color_image_ = std::move(color_image);
    } else {
      reportFailure("Could not load color image from file '%s'", maybe_filename->c_str());
    }
  }

  // Load depth image
  depth_image_ = std::nullopt;
  if (const auto maybe_filename = try_get_depth_filename()) {
    tried_to_load_image = true;
    Image1ui16 depth16;
    if (LoadPng(*maybe_filename, depth16)) {
      depth_image_ = Image1f(depth16.dimensions());
      ConvertUi16ToF32(depth16, *depth_image_, static_cast<float>(get_depth_scale()));
    } else {
      reportFailure("Could not load depth image from file '%s'", maybe_filename->c_str());
    }
  }

  // Load color image directory
  if (const auto maybe_pattern = try_get_color_glob_pattern()) {
    tried_to_load_image = true;
    for (const auto& filename : GlobFiles(*maybe_pattern, get_sort_by_number())) {
      Image3ub color_image;
      if (LoadImage(filename, color_image)) {
        color_images_.push_back(std::move(color_image));
      } else {
        reportFailure("Could not load color image from file '%s'", filename.c_str());
      }
    }
  }

  if (!tried_to_load_image) {
    reportFailure("Please specify path to image for loading");
  }

  // Publish images periodically
  tickPeriodically();
}

void ImageLoader::tick() {
  const int64_t acqtime = getTickTimestamp();

  if (color_image_) {
    Image3ub copy(color_image_->dimensions());
    Copy(*color_image_, copy);
    createAndPublishColorImageMessage(std::move(copy), acqtime);
  }

  if (depth_image_) {
    // Create message with depth image
    Image1f copy(depth_image_->dimensions());
    Copy(*depth_image_, copy);
    auto depth_proto = tx_depth().initProto();
    ToProto(std::move(copy), depth_proto.initDepthImage(), tx_depth().buffers());
    // Min/max depth
    depth_proto.setMinDepth(get_min_depth());
    depth_proto.setMaxDepth(get_max_depth());
    // Pinhole for depth camera
    auto pinhole = depth_proto.initPinhole();
    ToProto(get_focal_length(), pinhole.getFocal());
    ToProto(get_optical_center(), pinhole.getCenter());
    pinhole.setCols(depth_image_->cols());
    pinhole.setRows(depth_image_->rows());
    tx_depth().publish(acqtime);
  }

  if (!color_images_.empty()) {
    // Make sure the index is valid
    if (img_idx_ >= color_images_.size()) {
      if (get_loop_images()) {
        img_idx_ = 0;
      } else {
        img_idx_ = color_images_.size() - 1;
      }
    }

    const Image3ub& color_image = color_images_[img_idx_];
    Image3ub copy(color_image.dimensions());
    Copy(color_image, copy);
    createAndPublishColorImageMessage(std::move(copy), acqtime);
  }

  // Increment image index
  img_idx_ += 1;
}

}  // namespace message_generators
}  // namespace isaac
