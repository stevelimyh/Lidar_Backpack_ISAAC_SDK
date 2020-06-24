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

#include "open_pose_decoder_impl.hpp"

#include <algorithm>
#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace isaac {
namespace skeleton_pose_estimation {

void OpenPoseDecoderImpl::computeEdges(const TensorConstView3f& part_affinity_fields,
                                       const std::vector<PeakIDs>& part_peaks_map,
                                       std::vector<Edges>& edge_candidates) {
  // Go over possible edge candidates for each edge.  Each edge connects two parts, in turn
  // each part has a list of part peaks associated with it.  Edge candidates are formed
  // by all possible edges connecting items in these part peaks lists.
  for (size_t edge_id = 0; edge_id < edges_.size(); edge_id++) {
    std::vector<Edge> candidates;
    const PeakIDs& peak_a_ids = part_peaks_map[edges_[edge_id][0]];
    const PeakIDs& peak_b_ids = part_peaks_map[edges_[edge_id][1]];

    if (peak_a_ids.size() == 0 || peak_b_ids.size() == 0) {
      continue;
    }

    for (int peak_a_id : peak_a_ids) {
      for (int peak_b_id : peak_b_ids) {
        if (peak_a_id == peak_b_id) continue;
        const Peak& peak_a = peaks_[peak_a_id];
        const Peak& peak_b = peaks_[peak_b_id];

        // calculate edge direction of the edge candidate
        Vector2f edge_direction(peak_b.col - peak_a.col, peak_b.row - peak_a.row);
        const float edge_norm = edge_direction.norm();
        if (edge_norm < threshold_edge_size_) continue;
        edge_direction /= edge_norm;

        // calculate edge candidate score by matching the edge direction with the field direction
        int edge_sampling_counter = 0;
        float score_line_integral = 0.0f;

        // calculate step size along the line, sampling edge_sampling_steps_ samples
        const float step_col = (peak_b.col - peak_a.col) / static_cast<float>(edge_sampling_steps_);
        const float step_row = (peak_b.row - peak_a.row) / static_cast<float>(edge_sampling_steps_);

        for (int i = 0; i < edge_sampling_steps_; i++) {
          // calculate parts affinity field value at step i of line integral approximation
          const int location_col = std::lround(peak_a.col + i * step_col);
          const int location_row = std::lround(peak_a.row + i * step_row);
          Vector2f field(part_affinity_fields(location_row, location_col, edges_paf_[edge_id][0]),
                         part_affinity_fields(location_row, location_col, edges_paf_[edge_id][1]));

          const float score = edge_direction.dot(field);
          score_line_integral += score;
          if (score > threshold_edge_score_) edge_sampling_counter += 1;
        }

        // decrease the edge_score for edges longer than 50% of the width.
        const float edge_score = score_line_integral / edge_sampling_steps_
                      + std::min(.0f, .5f * part_affinity_fields.dimensions()[0] / edge_norm - 1.f);

        // threshold the edge score and add it into the edge candidates list
        if (edge_sampling_counter > threshold_edge_sampling_counter_ && edge_score > 0) {
          Edge candidate;
          candidate.peak_id1 = peak_a.peak_id;
          candidate.peak_id2 = peak_b.peak_id;
          candidate.score = edge_score;
          candidates.emplace_back(std::move(candidate));
        }
      }
    }

    // find the edges that maximize the total score (assignment algorithm)
    std::sort(candidates.begin(), candidates.end(),
              [](const Edge& a, const Edge& b) { return a.score > b.score; });
    for (const Edge& candidate : candidates) {
      bool assigned = false;
      for (const Edge& edge : edge_candidates[edge_id]) {
        if (edge.peak_id1 == candidate.peak_id1 || edge.peak_id2 == candidate.peak_id2) {
          assigned = true;
          break;
        }
      }
      if (assigned) continue;
      edge_candidates[edge_id].push_back(candidate);
    }
  }
}

void OpenPoseDecoderImpl::computeObjects(const std::vector<Edges>& edge_candidates,
                                         std::list<Object>& objects) {
  // The final step is to transform these detected edges into the final skeletons.
  // TODO: this doesn't seem optimal, profile it, try using a list, change algorithm if necessary.
  // NOTE: order of passing through edges likely changes the result.
  for (size_t edge_id = 0; edge_id < edges_.size(); edge_id++) {
    const int part_id1 = edges_[edge_id][0];
    const int part_id2 = edges_[edge_id][1];
    // at first, every edge belongs to a different object. then, if objects share
    // a part index with the same coordinates, they are the same object
    for (const Edge& edge : edge_candidates[edge_id]) {
      auto object_it1 = objects.end();
      auto object_it2 = objects.end();
      for (auto it = objects.begin(); it != objects.end(); ++it) {
        if (it->part_peak_ids[part_id1] == edge.peak_id1 ||
            it->part_peak_ids[part_id2] == edge.peak_id2) {
          if (object_it1 == objects.end()) {
            object_it1 = it;
          } else if (object_it2 == objects.end()) {
            object_it2 = it;
          }
        }
      }

      if (object_it1 == objects.end()) {  // ----------- found = 0, edge doesn't belong to an object
        if (edge_id < edges_.size() - 1) {  // TODO  investigate why do we have this if ?
          Object object;
          object.part_peak_ids.resize(num_parts_, -1);
          object.part_peak_ids[part_id1] = edge.peak_id1;
          object.part_peak_ids[part_id2] = edge.peak_id2;
          object.num_parts = 2;
          object.score = peaks_[edge.peak_id1].score + peaks_[edge.peak_id2].score + edge.score;
          objects.emplace_back(std::move(object));
        }
      } else if (object_it2 == objects.end()) {  // ---- found = 1, edge belongs to a single object
        if (object_it1->part_peak_ids[part_id2] == -1) {  // TODO  investigate  (!= edge.peak_id2)
          object_it1->part_peak_ids[part_id2] = edge.peak_id2;
          object_it1->num_parts += 1;
          object_it1->score += peaks_[edge.peak_id2].score + edge.score;
        }
      } else {  // ------------------------------------- found = 2, edge connects two objects
        int membership = 0;
        for (int part_id = 0; part_id < num_parts_; part_id++) {
          if (object_it1->part_peak_ids[part_id] >= 0 && object_it2->part_peak_ids[part_id] >= 0) {
            membership = 2;
            break;
          }
        }

        if (membership == 0) {
          // part_id belongs to previously unconnected objects, merge object_it2 into object_it1
          for (int part_id = 0; part_id < num_parts_; part_id++) {
            ASSERT((object_it1->part_peak_ids[part_id] == -1) ||
                   (object_it2->part_peak_ids[part_id] == -1), "Both shouldn't be initialized.");
            // Castling. This relies on '-1' initialization value of part_peak_ids and identities:
            // -1 += -1 + 1 = -1  ;  part_id += -1 + 1 = part_id  ;  -1 += part_id + 1 = part_id
            object_it1->part_peak_ids[part_id] += (object_it2->part_peak_ids[part_id] + 1);  // O-O
          }
          object_it1->num_parts += object_it2->num_parts;
          object_it1->score += object_it2->score;
          object_it1->score += edge.score;
          objects.erase(object_it2);
        } else {
          if (object_it1->part_peak_ids[part_id2] == -1) {  // TODO  investigate  (!= edge.peak_id2)
            object_it1->part_peak_ids[part_id2] = edge.peak_id2;
            object_it1->num_parts += 1;
            object_it1->score += peaks_[edge.peak_id2].score + edge.score;
          }
        }
      }
    }
  }

  // Merge objects that has common joints but don't have uncommon joints (merge `split` objects)
  for (auto object_it1 = objects.begin(); object_it1 != objects.end(); ++object_it1) {
    for (auto object_it2 = objects.begin(); object_it2 != object_it1; ++object_it2) {
      float objects_split_score = 0.0f;
      for (int part_id = 0; part_id < num_parts_; part_id++) {
        if (object_it1->part_peak_ids[part_id] == -1) continue;
        if (object_it2->part_peak_ids[part_id] == -1) continue;
        const Peak& peak_a = peaks_[object_it1->part_peak_ids[part_id]];
        const Peak& peak_b = peaks_[object_it2->part_peak_ids[part_id]];
        const Vector2f parts_distance(peak_b.col - peak_a.col, peak_b.row - peak_a.row);
        objects_split_score += parts_distance.norm();
      }

      // Detect object split
      if (objects_split_score > 0.0f && objects_split_score < threshold_split_score_) {
        // Merge these skeletons into a single skeleton, using joints from first skeleton
        for (int part_id = 0; part_id < num_parts_; part_id++) {
          if (   object_it1->part_peak_ids[part_id] != -1
              && object_it2->part_peak_ids[part_id] != -1) {
            object_it2->part_peak_ids[part_id] = -1;
            object_it2->num_parts -= 1;
          } else {
            // Castling. This relies on '-1' initialization value of part_peak_ids and identities:
            // -1 += -1 + 1 = -1  ;  part_id += -1 + 1 = part_id  ;  -1 += part_id + 1 = part_id
            object_it1->part_peak_ids[part_id] += (object_it2->part_peak_ids[part_id] + 1);  // O-O
          }
        }
        object_it1->num_parts += object_it2->num_parts;
        object_it1->score += object_it2->score;  // TODO: correct the score
        objects.erase(object_it2);
        break;
      }
    }
  }
}

void OpenPoseDecoderImpl::process(const TensorConstView3f& part_affinity_fields,
                                  const TensorConstView3f& gaussian_heatmap,
                                  const TensorConstView3f& maxpool_heatmap) {
  // FIXME, why num_parts_ != gaussian_heatmap.dimensions()[1], is it an 'not-a-part' channel ?
  ASSERT(num_parts_ <= static_cast<int>(gaussian_heatmap.dimensions()[2]),
         "Asserts parts number %d vs input tensor dimensions number %zd should be consistent.",
         num_parts_, gaussian_heatmap.dimensions()[2]);

  // Pre-precessing the peak map by filtering the values over a given threshold and
  // filling a vector from the Peak structure to make the candidates for each one of the parts
  // FIXME: This is likely expensive operation and a good candidate for optimization.
  std::vector<PeakIDs> part_peaks_map(num_parts_);
  int peak_id = 0;
  peaks_.clear();
  for (int row = 0; row < gaussian_heatmap.dimensions()[0]; row++) {
    for (int col = 0; col < gaussian_heatmap.dimensions()[1]; col++) {
      for (int part_id = 0; part_id < num_parts_; part_id++) {
        const float score = gaussian_heatmap(row, col, part_id);
        // FIXME - not sure '==' could be used for floats coming from GPU, even from maxpool
        if (score > threshold_heatmap_ && score == maxpool_heatmap(row, col, part_id)) {
          Peak info;
          info.peak_id = peak_id++;
          info.col = col;
          info.row = row;
          info.score = score;
          part_peaks_map[part_id].push_back(info.peak_id);
          peaks_.emplace_back(std::move(info));
        }
      }
    }
  }

  // Make a bipartite graph from object parts candidates by connecting them to form the edges
  std::vector<Edges> edge_candidates(edges_.size());  // edges indexed by edge_id
  computeEdges(part_affinity_fields, part_peaks_map, edge_candidates);

  // The final step is to transform these detected edges into the final skeletons.
  // TODO: this doesn't seem optimal, profile it, try using a list, change algorithm if necessary.
  // NOTE: order of passing through edges likely changes the result.
  std::list<Object> objects;
  computeObjects(edge_candidates, objects);

  // Populate objects_ array from objects list, according to the threshold
  objects_.clear();
  for (const auto& object : objects) {
    if (object.num_parts >= threshold_part_counter_ &&
        object.score / object.num_parts >= threshold_object_score_) {
      objects_.push_back(object);
    }
  }
}

void OpenPoseDecoderImpl::refinePeaks(const TensorConstView3f& gaussian_heatmap) {
  refined_peaks_.resize(peaks_.size());

  const int rows = gaussian_heatmap.dimensions()[0];
  const int cols = gaussian_heatmap.dimensions()[1];

  // INPUT:  X - gaussian heatmap peak location, in range [0...SIZE], HEATMAP - gaussian heatmap.
  // RESULT: REFINED_PEAK = SUM(HEATMAP[x] * x) / SUM(HEATMAP[x] FOR x in RANGE(X - 1, X + 1) + 0.5
  // Note, uniform kernel is used here. One step of gaussian mean shift could be good alternative.
  for (const auto& object : objects_) {
    for (int part_id = 0; part_id < num_parts_; part_id++) {
      const int peak_id = object.part_peak_ids[part_id];
      if (peak_id != -1) {
        const Peak& peak = peaks_[peak_id];
        RefinedPeak& refined_peak = refined_peaks_[peak_id];
        refined_peak.col = peak.col + refinement_col_bias_;
        refined_peak.row = peak.row + refinement_row_bias_;

        if (peak.col >= refinement_cols_ && peak.col < cols - refinement_cols_ &&
            peak.row >= refinement_rows_ && peak.row < rows - refinement_rows_) {
          // calculate "weighted coordinate" + refinement_bias_
          float weighted_peak_row = 0.0f;
          float weighted_peak_col = 0.0f;
          float weight_sum = 0.0f;
          for (int col = peak.col - refinement_cols_; col <= peak.col + refinement_cols_; ++col) {
            for (int row = peak.row - refinement_rows_; row <= peak.row + refinement_rows_; ++row) {
              const float weight = gaussian_heatmap(row, col, part_id);
              weighted_peak_row += weight * row;
              weighted_peak_col += weight * col;
              weight_sum += weight;
            }
          }
          if (!IsAlmostZero(weight_sum)) {
            refined_peak.row = weighted_peak_row / weight_sum + refinement_row_bias_;
            refined_peak.col = weighted_peak_col / weight_sum + refinement_col_bias_;
          }
        }
      }
    }
  }
}

}  // namespace skeleton_pose_estimation
}  // namespace isaac
