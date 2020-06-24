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

namespace isaac {

// A simple C++ codelet that prints periodically
class Ping : public alice::Codelet {
 public:
  // Has whatever needs to be run in the beginning of the program
  void start() override;
  // Has whatever needs to be run repeatedly
  void tick() override;

  // Message to be printed at every tick
  ISAAC_PARAM(std::string, message, "Hello World!");
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::Ping);
