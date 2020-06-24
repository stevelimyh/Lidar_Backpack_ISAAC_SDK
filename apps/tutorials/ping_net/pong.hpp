/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/ping.capnp.h"

class Pong : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // An incoming message channel on which we receive pings.
  ISAAC_PROTO_RX(PingProto, trigger);

  // Specifies how many times we print 'PONG' when we are triggered
  ISAAC_PARAM(int, count, 3);
};

ISAAC_ALICE_REGISTER_CODELET(Pong);
