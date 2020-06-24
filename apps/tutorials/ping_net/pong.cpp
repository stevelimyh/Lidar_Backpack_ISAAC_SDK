/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pong.hpp"

#include <string>

void Pong::start() {
  tickOnMessage(rx_trigger());
}

void Pong::tick() {
  // Parse the message we received
  auto proto = rx_trigger().getProto();
  const std::string message = proto.getMessage();

  // Print the desired number of 'PONG!' to the console
  const int num_beeps = get_count();
  LOG_DEBUG("%s:", message.c_str());
  for (int i = 0; i < num_beeps; i++) {
    LOG_DEBUG(" PONG!");
  }
  if (num_beeps > 0) {
    LOG_DEBUG("\n");
  }
}
