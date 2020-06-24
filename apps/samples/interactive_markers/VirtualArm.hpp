/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

// Testing codelet to let all the editable edges shown in the map and animate one of them
class VirtualArm : public isaac::alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Name of the reference frame of the left side of the pose
  ISAAC_PARAM(std::string, lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
};

ISAAC_ALICE_REGISTER_CODELET(VirtualArm);
