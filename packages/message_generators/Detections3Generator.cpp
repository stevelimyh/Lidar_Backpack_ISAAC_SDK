/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/message_generators/Detections3Generator.hpp"

#include "engine/alice/node.hpp"
#include "messages/geometry.hpp"

namespace isaac {
namespace message_generators {

void Detections3Generator::start() {
  tickPeriodically();
}

void Detections3Generator::tick() {
  // Initialize the output poses proto
  auto pose_builder = tx_output_poses().initProto();
  auto poses_builder = pose_builder.initPoses(get_num_detections());
  auto predictions_builder = pose_builder.initPredictions(get_num_detections());

  const double time = node()->clock()->time();
  // add translation to the object pose
  Pose3d object_T_object_new = get_pose();
  for (int detection = 0; detection < get_num_detections(); detection++) {
    const auto maybe = node()->pose().tryGet(get_reference(), get_label(), time);
    auto camera_T_object_new = *maybe * object_T_object_new;

    ToProto(camera_T_object_new, poses_builder[detection]);
    auto prediction = predictions_builder[detection];
    prediction.setLabel(get_label());
    prediction.setConfidence(1.0);
  }
  tx_output_poses().publish(time);
}

}  // namespace message_generators
}  // namespace isaac
