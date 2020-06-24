/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/ping.capnp.h"

class Ping : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  ISAAC_PARAM(std::string, message, "Hello World!");
  ISAAC_PROTO_TX(PingProto, ping);
};
ISAAC_ALICE_REGISTER_CODELET(Ping);
