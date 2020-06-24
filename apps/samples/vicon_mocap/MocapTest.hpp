/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "messages/marker_list.capnp.h"
#include "messages/pose_tree.capnp.h"

namespace isaac {
namespace vicon {

// The MocapTest codelet publishes the size of the pose tree to Websight along
// with a visualization of all the markers present in the scene.
class MocapTest : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // receiver message for the pose tree of the Vicon scene
  ISAAC_PROTO_RX(PoseTreeProto, vicon_pose_tree);
  // receiver message for all the markers in the Vicon scene
  ISAAC_PROTO_RX(MarkerListProto, vicon_markers);

  // Whether or not to connect all markers using line segments
  ISAAC_PARAM(bool, draw_line_segments, false)
};

}  // namespace vicon
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::vicon::MocapTest);
