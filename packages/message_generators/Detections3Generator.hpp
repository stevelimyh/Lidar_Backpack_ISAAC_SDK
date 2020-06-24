/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/detections.capnp.h"

namespace isaac {
namespace message_generators {

// Publishes Detections3Proto based on the pose from pose tree
// It also adds additional noise to perturb the output
class Detections3Generator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // publish dummy poses to test pose refinement
  ISAAC_PROTO_TX(Detections3Proto, output_poses);

  // Randomized pose to be added to dummy pose to generate different start positions
  ISAAC_PARAM(Pose3d, pose);
  // object label param for pose
  ISAAC_PARAM(std::string, label, "object");
  // reference label param for pose
  ISAAC_PARAM(std::string, reference, "camera");
  // Number of detections at output
  ISAAC_PARAM(int, num_detections, 1);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::Detections3Generator);
