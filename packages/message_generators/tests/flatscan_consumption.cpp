/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"
#include "packages/message_generators/FlatscanGenerator.hpp"

namespace isaac {

// Reads flatscan from FlatscanProto
class FlatscanConsumer : public alice::Codelet {
 public:
  ISAAC_PROTO_RX(FlatscanProto, flatscan);

  void start() override {
    tickOnMessage(rx_flatscan());
  }

  void tick() override {
    auto reader = rx_flatscan().getProto();
    auto msg_ranges = reader.getRanges();
    auto msg_angles = reader.getAngles();
    size_t expected_beam_count = get_expected_beam_count();
    EXPECT_EQ(msg_ranges.size(), expected_beam_count);
    EXPECT_EQ(msg_angles.size(), expected_beam_count);
    if (msg_ranges.size() != expected_beam_count || msg_angles.size() != expected_beam_count) {
      // stop ticking
      reportFailure();
      return;
    }
  }

  ISAAC_PARAM(int, expected_beam_count);
};

TEST(Flatscan, MultipleConsumers) {
  constexpr int kBeamCount = 1800;

  alice::Application app;

  auto* generator_node = app.createMessageNode("generator");
  auto* generator =
      generator_node->addComponent<message_generators::FlatscanGenerator>("FlatscanGenerator");
  generator->async_set_tick_period("1000Hz");
  generator->async_set_beam_count(kBeamCount);

  constexpr size_t num_consumers = 100;
  std::vector<FlatscanConsumer*> consumers;
  for (size_t index = 0; index < num_consumers; ++index) {
    auto* consumer_node = app.createMessageNode("viewer" + std::to_string(index));
    auto* consumer =
        consumer_node->addComponent<FlatscanConsumer>("FlatscanConsumer" + std::to_string(index));
    consumer->async_set_expected_beam_count(kBeamCount);
    Connect(generator->tx_flatscan(), consumer->rx_flatscan());
    consumers.push_back(consumer);
  }

  app.startWaitStop(60.0);
  for (auto consumer : consumers) {
    EXPECT_GE(consumer->getTickCount(), 2);
  }
}

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::FlatscanConsumer);
