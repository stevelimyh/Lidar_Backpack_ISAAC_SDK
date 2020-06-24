/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CudaImagePong.hpp"

#include <string>

#include "engine/core/image/image.hpp"
#include "messages/image.hpp"

namespace isaac {

void CudaImagePong::start() {
  // By using tickOnMessage instead of tickPeriodically we instruct the codelet to only tick when
  // a new message is received on the incoming data channel `trigger`.
  tickOnMessage(rx_pong());
}

void CudaImagePong::tick() {
  // This function will now only be executed whenever we receive a new message. This is guaranteed
  // by the Isaac Robot Engine.
  //
  // When receiving a data type with a buffer such as a tensor, image, or samplecloud, there are
  // helper functions FromProto that will create a constant view of the data.  If a mutable copy is
  // needed it is necessary to perform a copy from the view to the mutable instance.
  CudaImageConstView3f cuda_view;
  FromProto(rx_pong().getProto(), rx_pong().buffers(), cuda_view);
  std::printf("Cuda Image Receieved\n");
}

}  // namespace isaac
