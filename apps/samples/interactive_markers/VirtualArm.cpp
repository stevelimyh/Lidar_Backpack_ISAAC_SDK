/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "VirtualArm.hpp"

#include <string>
#include <vector>

#include "engine/alice/components/Pose.hpp"

void VirtualArm::start() {
  tickPeriodically();
}

void VirtualArm::tick() {
  const isaac::Pose3d shoulder_T_elbow{
      isaac::SO3d::FromAngleAxis(isaac::Pi<double> * (getTickCount() % 11) / 5.0, {0.0, 0.0, 1.0}),
      isaac::Vector3d(0.0, 0.0, 1.0)};
  node()->pose().set(get_lhs_frame(), get_rhs_frame(), shoulder_T_elbow, getTickTime());
}
