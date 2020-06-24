/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ping.hpp"

void Ping::start() {
  tickPeriodically();
}

void Ping::tick() {
  LOG_INFO("ping");
  // create and publish a ping message
  auto proto = tx_ping().initProto();
  proto.setMessage(get_message());
  tx_ping().publish();
}

void Ping::stop() {}
