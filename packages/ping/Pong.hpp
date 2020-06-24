#pragma once
#include "engine/alice/alice.hpp"
#include "messages/ping.capnp.h"

class Pong : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;

  ISAAC_PROTO_RX(PingProto, trigger);
  ISAAC_PARAM(int, count, 3);
};
ISAAC_ALICE_REGISTER_CODELET(Pong);