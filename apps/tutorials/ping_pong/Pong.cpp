/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Pong.hpp"

#include <string>

namespace isaac {

void Pong::start() {
  // By using tickOnMessage instead of tickPeriodically we instruct the codelet to only tick when
  // a new message is received on the incoming data channel `trigger`.
  tickOnMessage(rx_trigger());
}

void Pong::tick() {
  // This function will now only be executed whenever we receive a new message. This is guaranteed
  // by the Isaac Robot Engine.

  // Parse the message we received
  auto proto = rx_trigger().getProto();
  const std::string message = proto.getMessage();

  // Print the desired number of 'PONG!' to the console
  const int num_beeps = get_count();
  std::printf("%s:", message.c_str());
  for (int i = 0; i < num_beeps; i++) {
    std::printf(" PONG!");
  }
  if (num_beeps > 0) {
    std::printf("\n");
  }
}

}  // namespace isaac
