/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "RigidBody3GroupGenerator.hpp"

#include "messages/math.hpp"

namespace isaac {
namespace message_generators {

namespace {
// Number of bodies published by this codelet.
constexpr size_t kNumBodies = 1;
}  // namespace

void RigidBody3GroupGenerator::start() {
  tickPeriodically();
}

void RigidBody3GroupGenerator::tick() {
  // Publish a message with a single body using parameters
  auto group = tx_bodies().initProto();
  auto body = group.initBodies(kNumBodies)[0];
  ToProto(get_pose(), body.initRefTBody());
  ToProto(get_linear_velocity(), body.initLinearVelocity());
  ToProto(get_angular_velocity(), body.initAngularVelocity());
  ToProto(get_linear_acceleration(), body.initLinearAcceleration());
  ToProto(get_angular_acceleration(), body.initAngularAcceleration());
  ToProto(Vector3d::Constant(1.0), body.initScales());
  group.initNames(kNumBodies).set(0, get_body_name());
  group.setReferenceFrame(get_reference_frame());
  tx_bodies().publish();
}

}  // namespace message_generators
}  // namespace isaac
