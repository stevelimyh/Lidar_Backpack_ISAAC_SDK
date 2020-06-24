/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CudaImagePing.hpp"

#include <utility>

#include "engine/core/image/image.hpp"
#include "engine/core/logger.hpp"
#include "messages/image.hpp"

namespace isaac {

void CudaImagePing::start() {
  tickPeriodically();
}

// Create and publish a Cuda Image. This can also work for regular Images, Tensors, SampleClouds
// CudaTensors, CudaSampleClouds
void CudaImagePing::tick() {
  // Create a simple image to hold some data.
  CudaImage3f image;
  image.resize(10, 10);

  // Images, Tensors, and SampleClouds for both cuda and non cuda variants have
  // ToProto and FromProto helper messages that will pack the proto header and
  // create  a buffer to hold the associated data.
  //
  // Note: You must use std::move to relinquish ownership of the data.
  ToProto(std::move(image), tx_ping().initProto(), tx_ping().buffers());
  tx_ping().publish();

  LOG_INFO("Sent a Cuda Image");
}

}  // namespace isaac
