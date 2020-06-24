/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PoseGenerator.hpp"

#include "messages/math.hpp"

namespace isaac {
namespace message_generators {

void PoseGenerator::start() {
  pose_ = get_initial_pose();
  tickPeriodically();
}

void PoseGenerator::tick() {
  node()->pose().set(get_lhs_frame(), get_rhs_frame(), pose_, getTickTime());
  pose_ = pose_ * get_step();
}

}  // namespace message_generators
}  // namespace isaac
