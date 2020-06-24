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
#include "engine/gems/ml/bounding_box_detection.hpp"
#include "messages/detections.capnp.h"

namespace isaac {
namespace message_generators {

// Codelet to publish mock bounding box detections.
// It takes a user defined JSON which specifies the class and the bounding box rectangle coordinates
// for the detections and creates a Detections2Proto message out of it.
class Detections2Generator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output mocked detection proto message
  ISAAC_PROTO_TX(Detections2Proto, mock_detections);
  // Parameter defining the configuration of the detections we need to mock.
  // Format: [ { "class_label": "A", "confidence": 0.8,
  //             "bounding_box_coordinates": [0.0, 0.0, 100.0, 100.0] } ]
  // The bounding box coordinates are of the form (x1, y1, x2, y2)
  ISAAC_PARAM(nlohmann::json, detection_configuration, {});

 private:
  // List of bounding boxes that are to be published each tick
  std::vector<BoundingBoxDetection> mocked_detections_;
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::Detections2Generator);
