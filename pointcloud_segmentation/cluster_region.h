/*
 * Copyright 2023 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This ClusterRegion class is to segment the Lidar point cloud into several
// small clusters. By taking advantage of the distribution property of the
// Lidar data (range map), we collect the neighboring points into the same
// cluster by connected-components algorithm. This class is usually used after
// floor segmentation.
#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLUSTER_REGION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLUSTER_REGION_H_

#include <optional>
#include <queue>
#include <vector>

#include "absl/log/check.h"
#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/detected_objects.pb.h"
#include "pointcloud_segmentation/region_segmentation_config.pb.h"
#include "pointcloud_segmentation/segmentation.h"

namespace mobility {
namespace detail {

struct SourceCandidateIndexPair {
  int source_index;
  int candidate_index;
  SourceCandidateIndexPair(int source, int candidate)
      : source_index(source), candidate_index(candidate) {}
};

}  // namespace detail

template <typename PointT = eigenmath::Vector3f>
class ClusterRegion {
 public:
  using Config = ClusterRegionConfigProto;
  // Check if the ConfigProto has been initialized, if not use default
  // values.
  static void ApplyDefaultConfigValues(ClusterRegionConfigProto* config) {
    if (!config->has_min_region_inliers()) {
      config->set_min_region_inliers(7);
    }
    if (!config->has_squared_distance_threshold()) {
      config->set_squared_distance_threshold(1.0);
    }
    if (!config->has_half_search_window()) {
      config->set_half_search_window(1);
    }
  }

  using InputData = Cloud<PointT>;
  using NeighborType = detail::SourceCandidateIndexPair;

  explicit ClusterRegion(const ClusterRegionConfigProto& config)
      : config_(config) {
    ApplyDefaultConfigValues(&config_);
  }

  void Init(const InputData& in_data, int seed_point_index,
            std::queue<NeighborType>* unexamined_point_indices) {
    inlier_indices_.clear();

    // The very initial seed has no source point, so it's paired itself and
    // pushed into a queue.
    unexamined_point_indices->push(
        NeighborType(seed_point_index, seed_point_index));
  }

  // Check if the given candidate_idx is a cluster inlier. Returns true if it is
  // an inlier.
  bool CheckInlierAndUpdate(const InputData& in_data,
                            const NeighborType current_pair, Cloud<int>* labels,
                            int current_label_id) {
    const int candidate_index = current_pair.candidate_index;
    const int source_index = current_pair.source_index;

    const auto& candidate_point = in_data.AtUnsafe(candidate_index);
    const auto& source_point = in_data.AtUnsafe(source_index);
    // Check if we want to include this point in this cluster region.
    if ((candidate_point - source_point).squaredNorm() <
        config_.squared_distance_threshold()) {
      // Add the current candidate.
      inlier_indices_.push_back(candidate_index);
      // Write region label to this point.
      labels->AtUnsafe(candidate_index) = current_label_id;
      label_id_ = current_label_id;
      return true;
    }

    // Reset this label as unlabeled.
    labels->AtUnsafe(candidate_index) = kUnlabeled;
    return false;
  }

  // Does a final check, whether the cluster fulfills all quality criteria and
  // returns true if this is the case.
  bool CheckAndFinalize(const InputData& in_data, const Cloud<int>& labels) {
    if (inlier_indices_.size() < config_.min_region_inliers()) {
      return false;
    }
    return true;
  }

  // Add neighbors to unexamined_point_indices in the 8-neighborhood (default)
  // if "labels" indicates that the neighbor has not been added to
  // unexamined_point_indices yet. The user can define the size of the search
  // window.
  void AddNeighbors(const InputData& points, const NeighborType& point_pair,
                    Cloud<int>* labels,
                    std::queue<NeighborType>* unexamined_point_indices) {
    const int point_index = point_pair.candidate_index;
    int row, col;
    points.IndexToRowCol(point_index, &row, &col);
    const int half_window = config_.half_search_window();

    for (int delta_col = -half_window; delta_col <= half_window; ++delta_col) {
      for (int delta_row = -half_window; delta_row <= half_window;
           ++delta_row) {
        if (delta_col == 0 && delta_row == 0) {
          continue;
        }
        const int col_current = col + delta_col;
        const int row_current = row + delta_row;
        if (col_current < 0 || col_current >= points.Cols() ||
            row_current < 0 || row_current >= points.Rows() ||
            labels->AtUnsafe(row_current, col_current) != kUnlabeled) {
          continue;
        }

        labels->AtUnsafe(row_current, col_current) =
            kAlreadyInUnexaminedPointsQueue;
        unexamined_point_indices->push(NeighborType(
            point_index, labels->LinearizeIndex(row_current, col_current)));
      }
    }
  }

  static void CheckInputDataAndLabels(const InputData& points,
                                      Cloud<int>* labels) {
    CHECK_GT(points.Rows(), 1);
    CHECK(labels);

    CHECK_EQ(labels->Cols(), points.Cols());
    CHECK_EQ(labels->Rows(), points.Rows());
  }

  const std::vector<int>& inlier_indices() const { return inlier_indices_; }
  std::vector<int>* mutable_inlier_indices() { return &inlier_indices_; }
  const int label_id() const { return label_id_; }
  int* mutable_label_id() { return &label_id_; }
  const ClusterRegionConfigProto& config() const { return config_; }

  const absl::optional<eigenmath::Vector3f>& Seed() const { return seed_; }
  void SetSeed(const eigenmath::Vector3f& seed) { seed_ = seed; }

 private:
  int label_id_ = 0;
  Config config_;
  std::vector<int> inlier_indices_;
  // The seed point location of this object centroid.
  std::optional<eigenmath::Vector3f> seed_ = std::nullopt;
};

}  // namespace mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLUSTER_REGION_H_
