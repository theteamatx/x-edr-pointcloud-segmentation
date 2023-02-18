
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

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_SEGMENTATION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_SEGMENTATION_H_

#include <algorithm>
#include <map>
#include <queue>
#include <vector>

#include "absl/log/check.h"
#include "eigenmath/types.h"
#include "pointcloud_segmentation/cloud.h"

namespace blue::mobility {

// All labels >= 0 indicate an assigned region label. These cannot be claimed
// again. They can be used to index into the regions vector. All other negative
// special labels are described below:
// Label locations that are still available to claim by regions.
constexpr int kUnlabeled = -1;
// Label locations that cannot be claimed by regions (masked out).
constexpr int kMaskedOut = -2;
// Internal value to signal that a point is already in the unexamined queue and
// shall not be added again.
constexpr int kAlreadyInUnexaminedPointsQueue = -3;
// The points that are already examined.
constexpr int kAlreadyExamedPoint = -4;
// The points that are in ego region.
constexpr int kMaskedEgo = -5;

namespace detail {

template <typename PointT>
float SquaredDistance(const PointT& p1, const PointT& p2) {
  return (p1 - p2).squaredNorm();
}

template <typename PointT>
int GetNumPlaneSupportPoints(const eigenmath::Plane3f& plane,
                             const float max_plane_distance, const int row,
                             const int col, const int half_neighborhood_size,
                             const Cloud<PointT>& points) {
  int num_support_points = 0;
  const int min_row = std::max(0, row - half_neighborhood_size);
  const int max_row = std::min(points.Rows() - 1, row + half_neighborhood_size);
  const int min_col = std::max(0, col - half_neighborhood_size);
  const int max_col = std::min(points.Cols() - 1, col + half_neighborhood_size);
  for (int support_col = min_col; support_col <= max_col; ++support_col) {
    for (int support_row = min_row; support_row <= max_row; ++support_row) {
      if (plane.absDistance(points.AtUnsafe(support_col, support_row)) <
          max_plane_distance) {
        ++num_support_points;
      }
    }
  }
  return num_support_points;
}

// Computes the squared average normal length over a square neighborhood and
// updates the sliding window of row normal averages, assuming that the sliding
// window moves from left to right (increasing columns).
template <typename NormalT>
void GetSquaredAverageNormalLengthAndUpdateSlidingWindow(
    int row, int col, int neighborhood_size, const Cloud<NormalT>& normals,
    std::vector<eigenmath::Vector3f>* avg_row_normals,
    std::vector<int>* num_valid_row_normals, int* num_valid_normals,
    float* squared_normal_length) {
  const int kHalfNeighborhoodSize = neighborhood_size / 2;
  eigenmath::Vector3f avg_normal = eigenmath::Vector3f::Zero();
  *num_valid_normals = 0;

  // Update each row normal average by adding/subtracting entering/leaving
  // normals in the sliding window and accumulate vertically to finally
  // yield the average normal over the complete neighborhood
  for (int i = 0; i < neighborhood_size; ++i) {
    // Subtract the normal leaving the sliding window if it is valid
    if (col >= neighborhood_size) {
      const auto& leaving_normal = normals.AtUnsafe(
          col - neighborhood_size, row + i - kHalfNeighborhoodSize);
      if (leaving_normal.allFinite()) {
        (*avg_row_normals)[i] -= leaving_normal;
        --(*num_valid_row_normals)[i];
      }
    }

    // Add the normal entering the sliding window if it is valid
    const auto& entering_normal =
        normals.AtUnsafe(col, row + i - kHalfNeighborhoodSize);
    if (entering_normal.allFinite()) {
      (*avg_row_normals)[i] += entering_normal;
      ++(*num_valid_row_normals)[i];
    }

    // Update average normal by weighting the previous accumulation against
    // the number of added valid normals
    const int new_num_valid_normals =
        *num_valid_normals + (*num_valid_row_normals)[i];
    avg_normal = (*num_valid_normals * avg_normal +
                  (*num_valid_row_normals)[i] * (*avg_row_normals)[i]) /
                 new_num_valid_normals;
    *num_valid_normals = new_num_valid_normals;
  }
  *squared_normal_length = avg_normal.squaredNorm();
}

}  // namespace detail

// Prepare a segmentation labels marker from the given point cloud.
CloudBuffer<int> PrepareUnlabeledLabels(
    const Cloud<eigenmath::Vector3f>& point_cloud);

// Selects seed points by checking that the normals in a neighborhood are
// sufficiently parallel.
// To achieve this, the function evaluates a square neighborhood of size
// neighborhood_size and checks if the average normal length is larger than
// min_avg_normal_length and a minimum of min_num_valid_normals are in the
// current neighborhood. This average normal length is used as a measure for
// local planarity. The implementation uses a quadratic sliding window of size
// neighborhood_size to efficiently implement this.
template <typename PointT, typename NormalT>
std::vector<int> FindSeedPointsFromAverageNormals(
    const Cloud<PointT>& points, const Cloud<NormalT>& normals,
    const int neighborhood_size = 5, const int min_num_valid_normals = 8,
    const float min_avg_normal_length = 0.9999f) {
  CHECK_EQ(points.Cols(), normals.Cols());
  CHECK_EQ(points.Rows(), normals.Rows());
  CHECK_GT(points.Rows(), 1);
  const int kHalfNeighborhoodSize = neighborhood_size / 2;
  const float kSquaredMinAvgNormalLength =
      min_avg_normal_length * min_avg_normal_length;

  std::vector<int> seed_point_indices;

  if (points.Rows() < neighborhood_size || points.Cols() < neighborhood_size) {
    return seed_point_indices;
  }

  std::vector<eigenmath::Vector3f> avg_row_normals(neighborhood_size);
  std::vector<int> num_valid_row_normals(neighborhood_size);

  // Computes the average normal using sliding windows over neighborhood_size
  // rows and checks if it fulfills the seed point criteria
  for (int row = kHalfNeighborhoodSize;
       row < points.Rows() - kHalfNeighborhoodSize; ++row) {
    // Clear all row normal averages.
    for (int i = 0; i < neighborhood_size; ++i) {
      avg_row_normals[i] = eigenmath::Vector3f::Zero();
      num_valid_row_normals[i] = 0;
    }

    for (int col = 0; col < points.Cols(); ++col) {
      int num_valid_normals;
      float squared_normal_length;
      detail::GetSquaredAverageNormalLengthAndUpdateSlidingWindow(
          row, col, neighborhood_size, normals, &avg_row_normals,
          &num_valid_row_normals, &num_valid_normals, &squared_normal_length);

      // Decide whether we found a valid seed point
      if (col >= (neighborhood_size - 1) &&
          num_valid_normals >= min_num_valid_normals &&
          squared_normal_length >= kSquaredMinAvgNormalLength) {
        seed_point_indices.push_back(points.LinearizeIndex(row, col) -
                                     kHalfNeighborhoodSize);
      }
    }
  }
  return seed_point_indices;
}

// Selects seed points by checking that the plane of a seed point has enough
// (at least min_num_support_points) inliers in its plane model (plane distance
// must be smaller than max_plane_distance) in the NxN neighborhood
// (N == neighborhood_size)
template <typename PointT, typename NormalT>
std::vector<int> FindSeedPointsFromPlaneSupport(
    const Cloud<PointT>& points, const Cloud<NormalT>& normals,
    const int neighborhood_size = 9, const float max_plane_distance = 0.05f,
    const int min_num_support_points = 12) {
  CHECK_EQ(points.Cols(), normals.Cols());
  CHECK_EQ(points.Rows(), normals.Rows());
  CHECK_GT(points.Rows(), 1);
  const int kHalfNeighborhoodSize = neighborhood_size / 2;

  std::multimap<int, int> ranked_seed_point_indices;

  if (points.Rows() < neighborhood_size || points.Cols() < neighborhood_size) {
    return std::vector<int>();
  }

  for (int col = 0; col < points.Cols(); ++col) {
    for (int row = 0; row < points.Rows(); ++row) {
      const auto& point = points.AtUnsafe(col, row);
      const auto& normal = normals.AtUnsafe(col, row);
      if (!normal.allFinite() || !point.allFinite()) {
        continue;
      }
      eigenmath::Plane3f plane(normal, point);
      int num_support_points = detail::GetNumPlaneSupportPoints(
          plane, max_plane_distance, row, col, kHalfNeighborhoodSize, points);
      if (num_support_points >= min_num_support_points) {
        int seed_point_index = points.LinearizeIndex(row, col);
        ranked_seed_point_indices.emplace(num_support_points, seed_point_index);
      }
    }
  }
  // Build an ordered vector from the map.
  std::vector<int> seed_point_indices(ranked_seed_point_indices.size());
  auto it = ranked_seed_point_indices.begin();
  for (int& index : seed_point_indices) {
    index = it->second;
    ++it;
  }
  return seed_point_indices;
}

// Segments regions from the given points and normals by iteratively
// initializing regions from the given seed points (order last to first). Any
// point can at most belong to one region. The region_config are passed to the
// RegionT constructor parameterize the region growing behavior. Labels is
// an input/output parameter, it must have the same size as points. Only the
// points with label kUnlabeled are considered.

template <typename RegionT, typename RegionTConfigProto>
void SegmentRegions(const typename RegionT::InputData& input_data,
                    const std::vector<int>& seed_point_indices,
                    const RegionTConfigProto& region_config,
                    std::vector<RegionT>* regions, CloudBuffer<int>* labels,
                    int initial_id_offset = 0) {
  CHECK(regions);
  RegionT::CheckInputDataAndLabels(input_data, labels);

  regions->clear();
  std::queue<typename RegionT::NeighborType> unexamined_point_indices;
  RegionT region(region_config);

  int num_seed_points = seed_point_indices.size();

  while (num_seed_points > 0) {
    const int seed_point_index = seed_point_indices[num_seed_points - 1];
    --num_seed_points;
    // check if the seed point is already included in another region
    if (labels->AtUnsafe(seed_point_index) != kUnlabeled) {
      continue;
    }

    // Bootstrap region with seed point
    region.Init(input_data, seed_point_index, &unexamined_point_indices);

    while (!unexamined_point_indices.empty()) {
      const typename RegionT::NeighborType candidate =
          unexamined_point_indices.front();
      unexamined_point_indices.pop();

      if (region.CheckInlierAndUpdate(input_data, candidate, labels,
                                      regions->size() + initial_id_offset)) {
        region.AddNeighbors(input_data, candidate, labels,
                            &unexamined_point_indices);
      }
    }

    if (region.CheckAndFinalize(input_data, *labels)) {
      regions->push_back(region);
    } else {
      for (const int i : region.inlier_indices()) {
        labels->AtUnsafe(i) = kAlreadyExamedPoint;
      }
    }
  }

  // Reset these examed but non-segmented points back to unlabeled.
  for (int& label : *labels) {
    if (label == kAlreadyExamedPoint) {
      label = kUnlabeled;
    }
  }
}

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_SEGMENTATION_H_
