/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "OpenPoseDecoder.hpp"

#include <memory>
#include <string>
#include <vector>

#include "gems/open_pose_decoder_impl.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace skeleton_pose_estimation {

void OpenPoseDecoder::start() {
  decoder_impl_ = std::make_unique<OpenPoseDecoderImpl>(get_labels(), get_edges(), get_edges_paf(),
                                get_threshold_heatmap(), get_threshold_edge_size(),
                                get_threshold_edge_score(), get_threshold_edge_sampling_counter(),
                                get_threshold_part_counter(), get_threshold_object_score(),
                                get_threshold_split_score(), get_edge_sampling_steps());

  tickOnMessage(rx_part_affinity_fields());
  synchronize(rx_part_affinity_fields(), rx_gaussian_heatmap(), rx_maxpool_heatmap());
}

void OpenPoseDecoder::tick() {
  // for estimating key points
  const int64_t acqtime = rx_part_affinity_fields().acqtime();

  TensorConstView3f part_affinity_fields;
  TensorConstView3f gaussian_heatmap;
  TensorConstView3f maxpool_heatmap;
  if (!FromProto(rx_part_affinity_fields().getProto(), rx_part_affinity_fields().buffers(),
                 part_affinity_fields)) {
    reportFailure("Unable to read from part_affinity_fields tensor");
    return;
  }
  if (!FromProto(rx_gaussian_heatmap().getProto(), rx_gaussian_heatmap().buffers(),
                 gaussian_heatmap)) {
    reportFailure("Unable to read from gaussian_heatmap tensor");
    return;
  }
  if (!FromProto(rx_maxpool_heatmap().getProto(), rx_maxpool_heatmap().buffers(),
                 maxpool_heatmap)) {
    reportFailure("Unable to read from maxpool_heatmap tensor");
    return;
  }

  // Compute the peaks from gaussian heat map and the max pool tensor, process peaks and
  // part affinity fields to get object/parts 2D poses
  decoder_impl_->process(part_affinity_fields, gaussian_heatmap, maxpool_heatmap);

  // Refine peaks with "weighted coordinates" approach, for peaks encountered in objects_ array
  const bool refine_parts_coordinates = get_refine_parts_coordinates();
  if (refine_parts_coordinates) {
    decoder_impl_->refinePeaks(gaussian_heatmap);
  }

  // Populate the codelet output from decoder_impl_ state
  const std::vector<std::string> labels = get_labels();
  const std::string label = get_label();
  const std::vector<Vector2i> edges = get_edges();
  const Vector2d output_scale_factor = {get_output_scale()[0] / gaussian_heatmap.dimensions()[0],
                                        get_output_scale()[1] / gaussian_heatmap.dimensions()[1]};
  const int num_objects = decoder_impl_->getNumObjects();
  Skeleton2ListProto::Builder skeletons = tx_skeletons().initProto();
  skeletons.initSkeletons(num_objects);

  for (int object_id = 0; object_id < num_objects; ++object_id) {
    Skeleton2Proto::Builder skeleton = skeletons.getSkeletons()[object_id];
    PredictionProto::Builder prediction = skeleton.getLabel();
    prediction.setConfidence(decoder_impl_->getScore(object_id));
    prediction.setLabel(label);

    auto joints = skeleton.initJoints(labels.size());
    for (size_t part_idx = 0; part_idx < labels.size(); part_idx++) {
      const int peak_id = decoder_impl_->getPartPeakId(object_id, part_idx);
      PredictionProto::Builder label = joints[part_idx].initLabel();
      label.setLabel(labels[part_idx]);
      if (peak_id >= 0) {
        label.setConfidence(decoder_impl_->getPartScore(peak_id));
        Vector2dProto::Builder position = joints[part_idx].initPosition();
        if (refine_parts_coordinates) {
          position.setY(output_scale_factor[0] * decoder_impl_->getPartRefinedRow(peak_id));
          position.setX(output_scale_factor[1] * decoder_impl_->getPartRefinedCol(peak_id));
        } else {
          position.setY(output_scale_factor[0] * decoder_impl_->getPartRow(peak_id));
          position.setX(output_scale_factor[1] * decoder_impl_->getPartCol(peak_id));
        }
      } else {
        label.setConfidence(0);
      }
    }

    // Set edges (filtered by edges with source and target joints initialized)
    auto graph = skeleton.initGraph();
    auto is_connected = [&](const Vector2i& edge) { return   joints[edge[0]].hasPosition()
                                                          && joints[edge[1]].hasPosition(); };
    auto graph_edges = graph.initEdges(std::count_if(edges.begin(), edges.end(), is_connected));
    int edge_counter = 0;
    for (const Vector2i& edge : edges) {
      if (is_connected(edge)) {
        graph_edges[edge_counter].setSource(edge[0]);
        graph_edges[edge_counter].setTarget(edge[1]);
        edge_counter++;
      }
    }
  }
  tx_skeletons().publish(acqtime);
}

}  // namespace skeleton_pose_estimation
}  // namespace isaac
