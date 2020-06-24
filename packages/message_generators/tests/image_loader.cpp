/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <cstdlib>

#include "engine/gems/image/io.hpp"
#include "gtest/gtest.h"
#include "messages/camera.hpp"
#include "packages/message_generators/ImageLoader.hpp"

namespace isaac {

namespace {
void CreateImage() {
  Image3ub color(120, 160);
  for (int row = 0; row < color.rows(); row++) {
    for (int col = 0; col < color.cols(); col++) {
      color(row, col) = Pixel3ub(row, col, (row * col) % 256);
    }
  }
  SavePng(color, std::string(std::getenv("TEST_TMPDIR")) + "/color.png");
}
}  // namespace

// Writes dummy images to file and checks that loader loads them and publishes them
class ImageLoaderTest : public alice::Codelet {
 public:
  ISAAC_PROTO_RX(ColorCameraProto, color);
  ISAAC_PROTO_RX(DepthCameraProto, depth);

  void start() override {

    // TODO Add check for depth image

    tickOnMessage(rx_color());
  }

  void tick() override {
    auto input = rx_color().getProto();
    ImageConstView3ub color;
    bool ok = FromProto(input.getImage(), rx_color().buffers(), color);
    ASSERT_TRUE(ok);
    for (int row = 0; row < color.rows(); row++) {
      for (int col = 0; col < color.cols(); col++) {
        ASSERT_EQ(color(row, col)[0], row);
        ASSERT_EQ(color(row, col)[1], col);
        ASSERT_EQ(color(row, col)[2], (row * col) % 256);
      }
    }
  }
};

TEST(Navigation, TestNoGoalFrame) {
  CreateImage();
  alice::Application app;

  auto* loader_node = app.createMessageNode("loader");
  auto* loader = loader_node->addComponent<message_generators::ImageLoader>("loader");
  loader->async_set_color_filename(std::string(std::getenv("TEST_TMPDIR")) + "/color.png");
  loader->async_set_focal_length(Vector2d{60, 60});
  loader->async_set_optical_center(Vector2d{60, 80});
  loader->async_set_tick_period("0.1s");

  auto* test = app.createMessageNode("test")->addComponent<ImageLoaderTest>("test");
  Connect(loader->tx_color(), test->rx_color());

  app.startWaitStop(0.95);
  EXPECT_GE(test->getTickCount(), 2);
}

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ImageLoaderTest);
