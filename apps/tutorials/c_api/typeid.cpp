/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <cstdio>

#include "capnp/schema.h"

#include "messages/actor_group.capnp.h"
#include "messages/alice.capnp.h"
#include "messages/camera.capnp.h"
#include "messages/collision.capnp.h"
#include "messages/differential_base.capnp.h"
#include "messages/flatscan.capnp.h"
#include "messages/json.capnp.h"
#include "messages/pose_tree.capnp.h"
#include "messages/range_scan.capnp.h"
#include "messages/rigid_body_3_group.capnp.h"
#include "messages/segmentation_prediction.capnp.h"
#include "messages/state.capnp.h"

#define PRINT_MACRO(PROTO) \
  std::printf("Proto Id for " #PROTO " is %lu\n", ::capnp::typeId<PROTO>());

int main(int argc, char** argv) {
  PRINT_MACRO(ActorGroupProto)
  PRINT_MACRO(CollisionProto)
  PRINT_MACRO(ColorCameraProto)
  PRINT_MACRO(DepthCameraProto)
  PRINT_MACRO(SegmentationCameraProto)
  PRINT_MACRO(FlatscanProto)
  PRINT_MACRO(JsonProto)
  PRINT_MACRO(Plan2Proto)
  PRINT_MACRO(PoseTreeEdgeProto)
  PRINT_MACRO(RangeScanProto)
  PRINT_MACRO(RigidBody3GroupProto)
  PRINT_MACRO(StateProto)
}
