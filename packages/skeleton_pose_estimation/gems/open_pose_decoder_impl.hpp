/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/*
OpenPoseDecoderImpl code is refactored from
https://github.com/ildoonet/tf-pose-estimation
Copyright 2017 Ildoo Kim

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

https://github.com/ildoonet/tf-pose-estimation/blob/master/LICENSE
*/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {
namespace skeleton_pose_estimation {

// Part Affinity Fields processing. Estimates the 2D keypoints from DNN output
class OpenPoseDecoderImpl {
 public:
  OpenPoseDecoderImpl(const std::vector<std::string>& labels, const std::vector<Vector2i>& edges,
                      const std::vector<Vector2i>& edges_paf, const float threshold_heatmap,
                      const float threshold_edge_size, const float threshold_edge_score,
                      const int threshold_edge_sampling_counter, const int threshold_part_counter,
                      const float threshold_object_score, const float threshold_split_score,
                      const int edge_sampling_steps) :
    labels_(labels),
    num_parts_(labels.size()),
    edges_(edges),
    edges_paf_(edges_paf),
    threshold_heatmap_(threshold_heatmap),
    threshold_edge_size_(threshold_edge_size),
    threshold_edge_score_(threshold_edge_score),
    threshold_edge_sampling_counter_(threshold_edge_sampling_counter),
    threshold_part_counter_(threshold_part_counter),
    threshold_object_score_(threshold_object_score),
    threshold_split_score_(threshold_split_score),
    edge_sampling_steps_(edge_sampling_steps) {
  }

  // process() uses gaussian heat map and the max pool tensor to compute peakmap for detecting the
  // potential key points for each person in the frame and putting them as the vertex of a graph.
  // The graph edges are made based on the prior knowledge on the edges between object parts.
  // It uses the part_affinity_fields to make the graph weighted. The weighted graph shows all
  // possible edges between candidates of two parts. Then a greedy algorithm, specialized
  // to the task is used to find the optimum edges based on the maximum score that can be
  // obtained from the weights of the graph. For more details see: https://arxiv.org/abs/1812.08008
  //
  // Inputs:
  //  part_affinity_fields : field that indicates the direction of the edge between parts
  //  gaussian_heatmap : blurred heatmap of potential parts
  //  maxpool_heatmap : maxpool of blurred heatmap
  // Results:
  //  peaks_: a list of Peak structures, filtered candidates for each one of the object parts
  //  objects_:
  void process(const TensorConstView3f& part_affinity_fields,
               const TensorConstView3f& gaussian_heatmap,
               const TensorConstView3f& maxpool_heatmap);

  // Refines peaks_ by using a "weighted coordinate" approach. Final refined peaks output is a
  // floating point subpixel coordinate centered at the peak. Note, unlike the original
  // integer peaks, refined peaks are placed at grid centers (+0.5, +0.5) bias is added.
  // Inputs:
  //  gaussian_heatmap : blurred heatmap of potential parts
  //  peaks_ : list of peaks indexed by peak_id
  //  objects_ :  populated list of objects, each containing a connected graph of parts
  // Results:
  //  refined_peaks_ : populated array of refined peaks, indexed by peak_id
  void refinePeaks(const TensorConstView3f& gaussian_heatmap);

  // returns the number of objects_ in the current frame
  int getNumObjects() const { return objects_.size(); }

  // returns the number of predicted parts of the object in the current frame
  int getNumParts(int object_id) const {
    ASSERT(object_id < getNumObjects() && object_id >= 0, "Invalid object id");
    ASSERT(objects_[object_id].num_parts == std::count_if(objects_[object_id].part_peak_ids.begin(),
                                                          objects_[object_id].part_peak_ids.end(),
                                                          [](int peak_id) { return peak_id >= 0; }),
           "Number of counted num_parts should be equal to number of initialized parts.");
    return objects_[object_id].num_parts;
  }

  // returns the corresponding object id
  int getPartPeakId(int object_id, int part_id) const {
    return objects_[object_id].part_peak_ids[part_id];
  }

  // returns the score of the corresponding object id
  float getScore(int object_id) const {
    return objects_[object_id].score / objects_[object_id].num_parts;
  }
  // returns the peak column
  int getPartCol(int peak_id) const { return peaks_[peak_id].col; }
  // returns the peak row
  int getPartRow(int peak_id) const { return peaks_[peak_id].row; }
  // returns the refined peak column ("weighted coordinates" + refinement_col_bias_)
  float getPartRefinedCol(int peak_id) const { return refined_peaks_[peak_id].col; }
  // returns the refined peak row ("weighted coordinates" + refinement_row_bias_)
  float getPartRefinedRow(int peak_id) const { return refined_peaks_[peak_id].row; }
  // returns a candidate part score
  float getPartScore(int peak_id) const { return peaks_[peak_id].score; }

 private:
  // Structure to store peaks (local maxima) of the gaussian_heatmap
  struct Peak {
    // Column in the gaussian_heatmap tensor
    int col;
    // Row in the gaussian_heatmap tensor
    int row;
    // Value of the gaussian_heatmap tensor
    float score;
    // Unique index of the peak in the peaks_ array
    int peak_id;
  };

  // Structure to store refined peaks (local maxima) of the gaussian_heatmap
  struct RefinedPeak {
    // Refined column coordinate in the gaussian_heatmap tensor
    float col;
    // Refined row coordinate in the gaussian_heatmap tensor
    float row;
  };

  // Structure to store graph edges (and edges candidates)
  struct Edge {
    // Index of the peak in the peaks_ array
    int peak_id1;
    // Index of the peak in the peaks_ array
    int peak_id2;
    // Normalized and adjusted line integral of the edge direction with the parts affinity field
    float score;
  };

  // Structure to store final skeleton model objects
  struct Object {
    // Object's peak_ids indexed by part_id. Value of '-1' is used for missing or not detected parts
    std::vector<int> part_peak_ids;
    // Combined object score from gaussian_heatmap (peaks) and  part_affinity_fields (edges)
    float score;
    // Number of detected parts. Equal to number of non-negative values in the part_peak_ids list
    int num_parts;
  };

  // helper type to maintain lists of edges
  using Edges = std::vector<Edge>;

  // helper type to maintain lists of peak_ids
  using PeakIDs = std::vector<int>;

  // Make a bipartite graph from object parts candidates by connecting them to form the edges,
  // select these by matching directions of edge candidadate and parts affinity fields.
  //
  // Inputs:
  //  edges_ : template list of edges that describes the object connectivity
  //  peaks_ : list of peaks indexed by peak_id
  //  part_affinity_fields : field that indicates the direction of the edge between parts
  //  part_peaks_map : lists of peak_id for every part (indexed by part_id)
  // Results:
  //  edge_candidates : reference to a list of edge structures, indexed by edge_id
  void computeEdges(const TensorConstView3f& part_affinity_fields,
                    const std::vector<PeakIDs>& part_peaks_map,
                    std::vector<Edges>& edge_candidates);

  // Transforms detected edge candidates into the final skeletons with greedy graph expantion
  // Inputs:
  //  edges_     : template list of edges that describes the object connectivity
  //  edge_candidates : list of detected edge structures, indexed by edge_id
  // Results:
  //  objects :  populated list of objects, each containing a connected graph of parts
  void computeObjects(const std::vector<Edges>& edge_candidates, std::list<Object>& objects);

  // List of detected objects containing skeleton model part locations
  std::vector<Object> objects_;

  // A flattened list of peaks, by peak_id
  std::vector<Peak> peaks_;

  // A flattened list of refined peaks, by peak_id. The list is populated only for skeletons peaks.
  std::vector<RefinedPeak> refined_peaks_;

  // List of part (or keypoint) labels, ordered by indices matching the order of the network and
  // indices in edges vectors
  const std::vector<std::string> labels_;

  // Total possible number of parts (or keypoints). Equal to labels_.size()
  const int num_parts_;

  // List of edges of indices to keypoint labels of the skeleton model. Indexed by edge_id;
  const std::vector<Vector2i> edges_;

  // List of indices to parts affinity field for the components of the field. Indexed by edge_id.
  const std::vector<Vector2i> edges_paf_;

  // Peak-map preprocessing threshold part-candidates below this threshold are discarded
  const float threshold_heatmap_;

  // PAF-candidate edge size, edge-candidates below this threshold are discarded
  const float threshold_edge_size_;

  // PAF-candidate dot-product threshold edge-candidates below this threshold are discarded
  const float threshold_edge_score_;

  // PAF-candidate counter threshold edge-candidates below this threshold are discarded
  const int threshold_edge_sampling_counter_;

  // Final skeleton detection part counter threshold, detections with fewer parts are discarded
  const int threshold_part_counter_;

  // Final skeleton detection score threshold, detections with lower threshold are discarded
  const float threshold_object_score_;

  // Final skeleton detection split threshold, objects with lower threshold are not merged
  const float threshold_split_score_;

  // Number of sampling steps to calculate line integral over the part affinity field
  const int edge_sampling_steps_;

  // Number of columns over which to calculate "weighted coordinate" for peak refinement
  static constexpr int refinement_cols_ = 1;

  // A bias to add to column coordinate after the "weighted coordinate" peak refinement
  static constexpr float refinement_col_bias_ = 0.5;

  // Number of rows over which to calculate "weighted coordinate" for peak refinement
  static constexpr int refinement_rows_ = 1;

  // A bias to add to row coordinate after the "weighted coordinate" peak refinement
  static constexpr float refinement_row_bias_ = 0.5;
};

}  // namespace skeleton_pose_estimation
}  // namespace isaac
