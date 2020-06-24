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
#include "engine/core/math/types.hpp"

namespace isaac {
namespace message_generators {

// PoseGenerator creates a series of poses which moves by step every tick
class PoseGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Name of the reference frame of the left side of the pose
  ISAAC_PARAM(std::string, lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
  // Initial pose
  ISAAC_PARAM(Pose3d, initial_pose, Pose3d::Identity());
  // The pose delta for every tick
  ISAAC_PARAM(Pose3d, step, Pose3d::Translation({1.0, 0.0, 0.0}));

 private:
  Pose3d pose_;
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::PoseGenerator);
