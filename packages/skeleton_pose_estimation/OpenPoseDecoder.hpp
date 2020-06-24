/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "messages/detections.capnp.h"
#include "messages/tensor.capnp.h"

namespace isaac {
namespace skeleton_pose_estimation {

// Using the implementation in the gems/OpenPoseDecoderImpl
class OpenPoseDecoderImpl;

// OpenPoseDecoder converts a tensor from OpenPose-type model into a list of Skeleton models
// Note: Because a modified OpenPose architecture is used, tensors are not compatible
// with the original paper.
//
// OpenPose is a popular model architecture that allows 2D pose estimation of keypoints (or "parts")
// of articulate and solid objects. Examples of such objects include humans, vehicles,
// animals, and robotic arms. Only a single type of object is normally supported by the model;
// however, multiple instances of the object are supported
// Note: OpenPose performs simultaneous detection and 'skeleton model' pose estimation of objects.
// In the following documentation, 'objects', 'skeleton models', and 'skeletons' may be used.
// For more information about the model, please refer to https://arxiv.org/pdf/1812.08008.pdf
//
// OpenPoseDecoder takes in a multiple tensors from the Open Pose neural network. Specifically these
// tensors are used: Part Affinity Fields, Parts Gaussian Heatmaps, and Parts Gaussian Heatmaps
// MaxPool tensors.
// It uses Parts Gaussian Heatmaps and Parts Gaussian Heatmaps MaxPool to compute the PeakMap for
// detecting the potential key points for each object in the frame and outputs these keypoints
// as the vertex of a graph. The graph edges are made based on prior knowledge of the edges
// between object parts. It then uses the Part Affinity Fields tensor to make the graph weighted.
// The weighted graph contains all possible edges between candidates of two parts. Then a greedy
// algorithm specialized to the task is used to find the optimum edges based on the maximum score
// that can be obtained from the weights of the graph. It then refines the positions of final
// keypoints and publishes final graphs as a Skeleton2ListProto message.
class OpenPoseDecoder : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // [0] : part_affinity_fields : PAFLayer             = "lambda_2/conv2d_transpose"
  ISAAC_PROTO_RX(TensorProto, part_affinity_fields);
  // [1] : gaussian_heatmap     : GaussianHeatMapLayer = "lambda_3/tensBlur_depthwise_conv2d"
  ISAAC_PROTO_RX(TensorProto, gaussian_heatmap);
  // [2] : maxpool_heatmap      : MaxPoolGHMLayer      = "tensBlur/MaxPool"
  ISAAC_PROTO_RX(TensorProto, maxpool_heatmap);

  // A list of 2D pose estimations of skeleton models for detected objects (list of SkeletonProto).
  // See  SkeletonProto for more details.
  ISAAC_PROTO_TX(Skeleton2ListProto, skeletons);

  // A string to initialize the 'label' field of the output SkeletonProto object. It should be set
  // to match the type of object detected by the model (for example 'human').
  ISAAC_PARAM(std::string, label);

  // List of strings to use as detected joints labels.  For example:  ["Elbow", "Wrist", ...]
  // It is used to initialize the 'label' field of skeleton joints. Note, the order and size of
  // this list of labels should match that of the gaussian_heatmap tensor (channels dimension).
  ISAAC_PARAM(std::vector<std::string>, labels);

  // List of edges to detect (as edges of the skeleton model). Each edge is defined by a pair of
  // indices into the labels array specified by the 'labels' parameter.  Indices are zero-based.
  // For example [[0, 1], [2, 3]] will define two edges with the first edge "Elbow" - "Wrist".
  // This list is configured at the training time of the model.
  ISAAC_PARAM(std::vector<Vector2i>, edges);

  // List of indices to channels of the part_affinity_fields tensor, to locate components of the
  // parts affinity field. This list is 'indexed by edge_id' (so the order and size of this list
  // should match that of the edges parameter. This list is configured at the model training time.
  ISAAC_PARAM(std::vector<Vector2i>, edges_paf);

  // Peak-map preprocessing threshold. Part-candidates below this threshold are discarded.
  ISAAC_PARAM(float, threshold_heatmap);

  // PAF-candidate edge size. Connection-candidates below this threshold are discarded.
  ISAAC_PARAM(float, threshold_edge_size);

  // PAF-candidate dot-product threshold. Connection-candidates below this threshold are discarded.
  ISAAC_PARAM(float, threshold_edge_score);

  // PAF-candidate counter threshold. Connection-candidates below this threshold are discarded.
  // Number of times dot-product was larger than threshold_edge_score during edge_sampling_steps
  // Note, it depends on edge_sampling_steps (should be smaller or equal to edge_sampling_steps).
  ISAAC_PARAM(int, threshold_edge_sampling_counter);

  // Final skeleton detection part counter threshold. Detections with fewer parts are discarded.
  ISAAC_PARAM(int, threshold_part_counter);

  // Final skeleton detection score threshold. Detections with lower threshold are discarded.
  ISAAC_PARAM(float, threshold_object_score);

  // Final skeleton detection split threshold, objects with lower threshold are not merged.
  ISAAC_PARAM(float, threshold_split_score);

  // Number of sampling steps to calculate line integral over the part affinity field.
  // Note also: threshold_edge_sampling_counter.
  ISAAC_PARAM(int, edge_sampling_steps);

  // Refine peaks of gaussian heatmap with "weighted coordinates" approach. The gaussian heatmap
  // grid cells of adjacent to the initial peak are used to refine the peak position to get better
  // estimates of parts coordinates.  Note, the output of "refined parts coordinates" are floating
  // point subpixel coordinates placed at "grid centers", rather than integer rows and columns.
  ISAAC_PARAM(bool, refine_parts_coordinates);

  // Output scale for the decoded skeleton pose output. For example, this could be
  // the image resolution (before downscaling to fit the network input tensor resolution).
  // The format is [output_scale_rows, output_scale_cols]
  ISAAC_PARAM(Vector2d, output_scale);

 private:
  // Part Affinity Fields processing. Estimates the 2D keypoints from DNN output.
  std::unique_ptr<OpenPoseDecoderImpl> decoder_impl_;
};

}  // namespace skeleton_pose_estimation
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::skeleton_pose_estimation::OpenPoseDecoder);
