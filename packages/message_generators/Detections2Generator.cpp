/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>
#include <vector>

#include "Detections2Generator.hpp"

#include "engine/gems/geometry/n_cuboid.hpp"
#include "messages/geometry.hpp"

namespace isaac {
namespace message_generators {

namespace {

// Parses information from a JSON containing the configuration of the mock detections.
// Stores the extracted information in a vector of bounding boxes.
void ParseDetections(const nlohmann::json& json, std::vector<BoundingBoxDetection>& detections) {
  ASSERT(json.is_array(), "Ill-formed detection info list");
  for (const auto& info : json) {
    BoundingBoxDetection detection_info;
    detection_info.class_name = info["class_label"];
    detection_info.probability = info["confidence"];
    // Make sure we have four values specifying the bounding box coordinates ( x1, y1, x2 and y2 )
    const auto& bbox_coordinates = info["bounding_box_coordinates"];
    ASSERT(bbox_coordinates.size() == 4,
           "Must provide four values denoting bounding box coordinates, in the order "
           "[min_x, min_y, max_x, max_y]");
    detection_info.bounding_box = geometry::RectangleD::FromOppositeCorners(
        Vector2d(bbox_coordinates[0].get<double>(), bbox_coordinates[1].get<double>()),
        Vector2d(bbox_coordinates[2].get<double>(), bbox_coordinates[3].get<double>()));
    detections.push_back(detection_info);
  }
}

}  // namespace

void Detections2Generator::start() {
  // Read the configuration for the mock detections
  ParseDetections(get_detection_configuration(), mocked_detections_);
  // Tick periodically
  tickPeriodically();
}

void Detections2Generator::tick() {
  // Initalize output proto
  auto detections_proto = tx_mock_detections().initProto();
  // Initialize sizes of the prediction list and bounding box list to the required size
  auto boxes_proto = detections_proto.initBoundingBoxes(mocked_detections_.size());
  auto predictions_proto = detections_proto.initPredictions(mocked_detections_.size());
  // Populate bounding box and class information in the output proto
  for (size_t i = 0; i < mocked_detections_.size(); i++) {
    // Populate label and confidence
    predictions_proto[i].setLabel(mocked_detections_[i].class_name);
    predictions_proto[i].setConfidence(mocked_detections_[i].probability);
    // Populate bounding box rectangle information
    ToProto(mocked_detections_[i].bounding_box, boxes_proto[i]);
  }
  // Publish mock detections
  tx_mock_detections().publish();
}

}  // namespace message_generators
}  // namespace isaac
