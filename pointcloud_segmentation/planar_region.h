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

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANAR_REGION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANAR_REGION_H_

#include <cmath>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "eigenmath/scalar_utils.h"
#include "eigenmath/types.h"
#include "pointcloud_segmentation/algorithms.h"
#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/plane_estimator.h"
#include "pointcloud_segmentation/region_segmentation_config.pb.h"
#include "pointcloud_segmentation/segmentation.h"
#include "pointcloud_segmentation/semantic_types.h"

namespace mobility {

enum class PlaneClass : int { kUnknown = 0, kFloor, kWall, kTable };

namespace detail {

enum NeighborhoodSelection { kUse8Neighborhood, kUse4Neighborhood };

struct Neighbor {
  int delta_x;
  int delta_y;
  int delta_index;  // = delta_x * height + delta_y: pre-calculated
};

// Returns either a vector with a 4-neighborhood or an 8-neighborhood.
std::vector<Neighbor> CreateNeighborhood(
    NeighborhoodSelection neighborhood_selection, int rows, int cols);

// Returns the next boundary point direction index in the range of
// [0, directions.size()) or -1 if no boundary point was found.
int FindNextBoundaryPointDirection(const Cloud<int>& labels,
                                   const int region_label,
                                   const std::vector<Neighbor>& directions,
                                   const int curr_x, const int curr_y,
                                   const int curr_idx, const int direction_idx);

// Returns a direction index to a neighbor that does not belong to the region
// indicated by region_label in the range of [0, directions.size()) or -1 if
// the given point is not on the boundary.
int FindInitialPredecessorDirection(const Cloud<int>& labels,
                                    const int region_label,
                                    const std::vector<Neighbor>& directions,
                                    const int curr_x, const int curr_y,
                                    const int curr_idx);

// Checks if the indices vector contains coordinates with a spread of at least
// min_cols and min_rows.
bool CheckMinRowsAndCols(const std::vector<int>& indices, int rows, int cols,
                         int min_cols, int min_rows);

// Returns true if the neighbor cell is within bounds of the point cloud.
bool InBounds(const Cloud<int>& labels, const int idx, const Neighbor& n);

}  // namespace detail

// PlanarRegion implements a region growing model for planar surfaces that
// initializes a plane from a seed point and then grows the inlier set based
// on a normal compatibility and plane distance check. The model is occasionally
// reestimated from the current inlier set to iteratively adapt.
template <typename PointT = eigenmath::Vector3f,
          typename NormalT = eigenmath::Vector3f>
class PlanarRegion {
 public:
  // Check if the ConfigProto has been initialized, if not use default
  // values.
  static void ApplyDefaultConfigValues(PlanarRegionConfigProto* config) {
    if (!config->has_max_plane_distance()) {
      config->set_max_plane_distance(0.05);
    }
    if (!config->has_min_region_area()) {
      config->set_min_region_area(0.05);
    }
    if (!config->has_min_region_inliers()) {
      config->set_min_region_inliers(5);
    }
    if (!config->has_plane_model_reestimation_period()) {
      config->set_plane_model_reestimation_period(30);
    }
    if (!config->has_discontinuity_min_range()) {
      config->set_discontinuity_min_range(1.2);
    }
    if (!config->has_discontinuity_max_range()) {
      config->set_discontinuity_max_range(4.0);
    }
    if (!config->has_discontinuity_normal_angle_diff()) {
      config->set_discontinuity_normal_angle_diff(5.0);
    }
    if (!config->has_discontinuity_z_diff()) {
      config->set_discontinuity_z_diff(0.05);
    }
    if (!config->has_discontinuity_z_ratio()) {
      config->set_discontinuity_z_ratio(0.7);
    }
  }

  struct InputData {
    // The inputData contains points coordinates and normals.
    const eigenmath::Pose3f robot_pose_point_cloud;
    CloudView<const PointT> points;
    CloudView<const NormalT> normals;
    InputData(const eigenmath::Pose3f in_robot_pose_point_cloud,
              CloudView<const PointT> in_points,
              CloudView<const NormalT> in_normals)
        : robot_pose_point_cloud(in_robot_pose_point_cloud),
          points(in_points),
          normals(in_normals) {}
  };

  using NeighborType = int;

  explicit PlanarRegion(const PlanarRegionConfigProto& config)
      : config_(config), projected_boundary_points_(0, 0) {
    ApplyDefaultConfigValues(&config_);
  }

  // Initialize the region from the given seed point
  void Init(const InputData& in_data, int seed_point_index,
            std::queue<NeighborType>* unexamined_point_indices) {
    const auto& seed_normal = in_data.normals.AtUnsafe(seed_point_index);
    const auto& seed_point = in_data.points.AtUnsafe(seed_point_index);
    seed_point_index_ = seed_point_index;
    plane_ = eigenmath::Plane3f(seed_normal, seed_point);

    plane_estimator_.Clear();
    plane_estimator_.SetNormalOrientation(seed_normal);
    centroid_ = seed_point;
    area_ = 0.0f;
    inlier_indices_.clear();
    projected_boundary_points_.Clear();

    unexamined_point_indices->push(seed_point_index);
  }

  // Check if a given candidate_idx is a region inlier. Returns true if it is an
  // inlier. This also occasionally updates the region model from all inliers.
  bool CheckInlierAndUpdate(const InputData& in_data,
                            const NeighborType candidate_index,
                            Cloud<int>* labels, int current_label_id) {
    const auto& point = in_data.points.AtUnsafe(candidate_index);
    // Check if we want to include this point in our planar region
    if (plane_.absDistance(point) < config_.max_plane_distance()) {
      inlier_indices_.push_back(candidate_index);
      plane_estimator_.AddPoint(point);

      if (inlier_indices_.size() % config_.plane_model_reestimation_period() ==
          0) {
        // Get a reestimated plane from the estimator.
        plane_ = plane_estimator_.Plane();
      }
      labels->AtUnsafe(candidate_index) = current_label_id;
      label_id_ = current_label_id;
      return true;
    } else {
      labels->AtUnsafe(candidate_index) = kUnlabeled;
      return false;
    }
  }

  // Does a final check, whether the region fulfills all quality criteria and
  // returns true if this is the case. Also performs other one-time tasks like
  // a final plane reestimation and boundary extraction to finalize the region.
  bool CheckAndFinalize(const InputData& in_data, const Cloud<int>& labels) {
    if (inlier_indices_.size() < config_.min_region_inliers()) {
      return false;
    }

    // Get the updated plane model one last time
    plane_ = plane_estimator_.Plane();
    centroid_ = plane_estimator_.Centroid();

    for (int i = inlier_indices_.size() - 1; i >= 0; --i) {
      if (FindLabeledRegionBoundary(inlier_indices_[i], in_data, labels,
                                    detail::kUse8Neighborhood)) {
        break;
      }
    }

    if (!detail::CheckMinRowsAndCols(boundary_indices_, labels.Rows(),
                                     labels.Cols(), 3, 3)) {
      return false;
    }

    if (boundary_indices_.empty()) {
      return false;
    }

    PlanarConvexHull(in_data.points, boundary_indices_, plane_,
                     &projected_boundary_points_);

    if (projected_boundary_points_.Size() < 3) {
      return false;
    }

    area_ = PolygonArea(in_data.points, boundary_indices_);

    return area_ >= config_.min_region_area();
  }

  // Add neighbors to unexamined_point_indices in the 4-neighborhood if labels
  // indicates that the neighbor has not been added to unexamined_point_indices
  // yet.
  void AddNeighbors(const InputData& in_data, const NeighborType& point_index,
                    Cloud<int>* labels,
                    std::queue<int>* unexamined_point_indices) {
    const auto& points = in_data.points;
    int row, col;
    points.IndexToRowCol(point_index, &row, &col);

    for (int delta_col = -1; delta_col <= 1; ++delta_col) {
      for (int delta_row = -1; delta_row <= 1; ++delta_row) {
        if (std::abs(delta_col + delta_row) != 1) {
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
        unexamined_point_indices->push(
            labels->LinearizeIndex(row_current, col_current));
      }
    }
  }

  static void CheckInputDataAndLabels(const InputData& in_data,
                                      Cloud<int>* labels) {
    const auto& points = in_data.points;
    const auto& normals = in_data.normals;

    CHECK_EQ(points.Cols(), normals.Cols());
    CHECK_EQ(points.Rows(), normals.Rows());
    CHECK_GT(points.Rows(), 1);
    CHECK(labels);
    CHECK_EQ(points.Cols(), labels->Cols());
    CHECK_EQ(points.Rows(), labels->Rows());
  }

  std::string PlaneClassName() const {
    switch (plane_class()) {
      case PlaneClass::kFloor:
        return semantic::kFloor;
      case PlaneClass::kWall:
        return semantic::kWall;
      case PlaneClass::kTable:
        return semantic::kTable;
      case PlaneClass::kUnknown:
        return semantic::kUnknownClass;
    }
    return semantic::kUnknownClass;
  }

  // Extracts the region boundary indices starting from start_idx. Returns false
  // if the point under start_idx is not a boundary point. The variable
  // neighborhood_selection can be set to kUse8Neighborhood or kUse4Neighborhood
  // for it to use the 8 or 4 neighboring points respectively for boundary
  // traversal.This function correctly deals with cases when start_idx is on a
  // one pixel wide branch of the boundary that has to be traversed forth and
  // back, visiting the start_idx twice instead of just once. In addition to the
  // boundary points for the planar region, it also computes discontinuous
  // boundary points based on certain geometric heuristics. These discontinuous
  // boundary points are used to mark obstacles, as 2 neighboring floor planes
  // could have a cliff edge.
  bool FindLabeledRegionBoundary(
      int start_idx, const InputData& in_data, const Cloud<int>& labels,
      detail::NeighborhoodSelection neighborhood_selection) {
    // Fill lookup table for next points to visit.
    const std::vector<detail::Neighbor> directions = detail::CreateNeighborhood(
        neighborhood_selection, labels.Rows(), labels.Cols());

    boundary_indices_.clear();
    discontinuous_boundary_indices_.clear();

    int curr_idx = start_idx;
    int curr_x = start_idx / labels.Rows();
    int curr_y = start_idx % labels.Rows();
    const int region_label = labels.AtUnsafe(start_idx);

    // Find one pixel with another label in the neighborhood -> assume thats the
    // one we came from.
    int direction_idx = detail::FindInitialPredecessorDirection(
        labels, region_label, directions, curr_x, curr_y, curr_idx);

    // No connection to outer regions => start_idx is not on the border.
    if (direction_idx < 0) {
      return false;
    }

    float discontinuity_min_range_sq =
        eigenmath::Square(config_.discontinuity_min_range());
    float discontinuity_max_range_sq =
        eigenmath::Square(config_.discontinuity_max_range());
    bool found_start_idx = false;
    while (true) {
      const int new_direction_index = detail::FindNextBoundaryPointDirection(
          labels, region_label, directions, curr_x, curr_y, curr_idx,
          direction_idx);
      CHECK_GE(new_direction_index, 0);

      // Update our current boundary point.
      direction_idx =
          (new_direction_index + directions.size() / 2) % directions.size();
      curr_idx += directions[new_direction_index].delta_index;
      curr_x += directions[new_direction_index].delta_x;
      curr_y += directions[new_direction_index].delta_y;

      // If we found the start index, there are two cases:
      // 1. We are done, when we would choose the first index as next index
      // 2. We have to go further, because we would take a different neighbor
      //    since we are apparently on a "one-pixel-wide" extremety of the
      //    region
      if (found_start_idx) {
        // Check if we would do another loop or take a different route.
        if (curr_idx == boundary_indices_.front()) {
          // Found the end as we would do the same loop.
          break;
        } else {
          // Go past the start index for another loop.
          found_start_idx = false;
        }
      }
      boundary_indices_.push_back(curr_idx);

      // Check if there is a geometric discontinuity between current point & its
      // neighbors in other regions.

      // Only for points within 4 meters of the robot and outside the first
      // couple of CBr rings to avoid noise due to skewing points.
      if (in_data.points.AtUnsafe(curr_idx).squaredNorm() >
              discontinuity_min_range_sq &&
          in_data.points.AtUnsafe(curr_idx).squaredNorm() <
              discontinuity_max_range_sq) {
        // Use 4 neighborhood.
        const std::vector<detail::Neighbor> directions_disc =
            detail::CreateNeighborhood(detail::kUse4Neighborhood, labels.Rows(),
                                       labels.Cols());
        if (std::count_if(directions_disc.begin(), directions_disc.end(),
                          [&](detail::Neighbor n) {
                            return detail::InBounds(labels, curr_idx, n);
                          }) == 4) {
          for (detail::Neighbor n : directions_disc) {
            // Skip this point & neighbor is any of the returns are nan.
            if (std::isnan(in_data.points.AtUnsafe(curr_idx).z()) ||
                std::isnan(
                    in_data.points.AtUnsafe(curr_idx + n.delta_index).z())) {
              continue;
            }
            // Make sure neighbor is in bounds and has a different label.
            if (!detail::InBounds(labels, curr_idx, n) ||
                region_label != labels.AtUnsafe(curr_idx + n.delta_index)) {
              continue;
            }

            // Do a geometric discontinuity check.
            const PointT pt_delta =
                in_data.points.AtUnsafe(curr_idx) -
                in_data.points.AtUnsafe(curr_idx + n.delta_index);
            const PointT pt_delta_robot_frame =
                in_data.robot_pose_point_cloud.quaternion() * pt_delta;

            const float cos_max_delta_angle_normals =
                in_data.normals.AtUnsafe(curr_idx).dot(
                    in_data.normals.AtUnsafe(curr_idx + n.delta_index));

            // Check for a minimum delta z or a delta normal angle difference to
            // qualify as discontinuous.
            if (std::abs(eigenmath::Degrees(
                    std::acos(cos_max_delta_angle_normals))) <
                    config_.discontinuity_normal_angle_diff() &&
                std::abs(pt_delta_robot_frame.z()) <
                    config_.discontinuity_z_diff()) {
              continue;
            }
            // Conditional to check that most discontinuity is in the z
            // dimension rather than the shadow line when an obstacle is
            // occluding the floor.
            if (std::abs(pt_delta_robot_frame.z()) /
                    pt_delta_robot_frame.norm() <
                config_.discontinuity_z_ratio()) {
              continue;
            }
            // If all checks are satisfied, add this point as discontinuous
            discontinuous_boundary_indices_.insert(curr_idx);
          }
        }
      }
      if (curr_idx == start_idx) {
        // Check our termination criterion the next time around.
        found_start_idx = true;
      }
    }

    return true;
  }

  const PlanarRegionConfigProto& config() const { return config_; }
  const eigenmath::Vector3f& centroid() const { return centroid_; }
  const eigenmath::Plane3f& plane() const { return plane_; }
  const PlaneEstimator& plane_estimator() const { return plane_estimator_; }
  int seed_point_index() const { return seed_point_index_; }
  float area() const { return area_; }
  const std::vector<int>& inlier_indices() const { return inlier_indices_; }
  const std::vector<int>& boundary_indices() const { return boundary_indices_; }
  const absl::flat_hash_set<int>& discontinuous_boundary_indices() const {
    return discontinuous_boundary_indices_;
  }
  int label_id() const { return label_id_; }
  const Cloud<eigenmath::Vector3f>& projected_boundary_points() const {
    return projected_boundary_points_;
  }
  PlaneClass plane_class() const { return plane_class_; }
  PlaneClass* mutable_plane_class() { return &plane_class_; }
  eigenmath::Vector3f* mutable_centroid() { return &centroid_; }
  eigenmath::Plane3f* mutable_plane() { return &plane_; }
  int* mutable_seed_point_index() { return &seed_point_index_; }
  std::vector<int>* MutableInlierIndicesForTestOnly() {
    return &inlier_indices_;
  }
  int* mutable_label_id() { return &label_id_; }

 private:
  int label_id_ = 0;
  PlanarRegionConfigProto config_;
  PlaneEstimator plane_estimator_;
  eigenmath::Vector3f centroid_;
  eigenmath::Plane3f plane_;
  int seed_point_index_;
  float area_;
  std::vector<int> inlier_indices_;
  std::vector<int> boundary_indices_;
  absl::flat_hash_set<int> discontinuous_boundary_indices_;
  CloudBuffer<eigenmath::Vector3f> projected_boundary_points_;
  PlaneClass plane_class_ = PlaneClass::kUnknown;
};

// Finds seed points near the centroids of the set of previous planar regions.
// These seed points are usually much better (for static objects) than what
// could be found by other simple methods like checking the local planarity. In
// order to account for sensor motion, one can set current_pose_prev equal to
// the motion the sensor underwent between the previous planar region extraction
// and the current. max_distance determines how far the seed points can be from
// the previous region centroid and max_normal_difference_angle determines how
// much the seed point normal can deviate from the previous region plane normal.
// All found seed points are added to seed_point_indices (it is not cleared in
// this function). points and normals represent the current point cloud that
// seed points shall be found in.
template <typename PointT, typename NormalT>
void FindSeedPointsFromLastPlanarRegions(
    const Cloud<PointT>& points, const Cloud<NormalT>& normals,
    const std::vector<PlanarRegion<PointT, NormalT>>& prev_regions,
    const eigenmath::Pose3f& current_pose_prev, const float max_distance,
    const float max_normal_difference_angle,
    std::vector<int>* seed_point_indices) {
  const float kMaxSquaredDistance = max_distance * max_distance;
  const float kCosMaxNormalDifferenceAngle = cosf(max_normal_difference_angle);
  struct SeedPoint {
    eigenmath::Vector3f point;
    eigenmath::Vector3f normal;
  };
  std::multimap<int, SeedPoint> past_region_centroids;
  for (int i = 0; i < prev_regions.size(); ++i) {
    past_region_centroids.emplace(
        prev_regions[i].inlier_indices().size(),
        SeedPoint{
            current_pose_prev * prev_regions[i].centroid(),
            current_pose_prev.quaternion() * prev_regions[i].plane().normal()});
  }
  // Create seed points from past region centroids by finding a matching
  // point index in the current point cloud
  for (const auto& [num_points, seed_point] : past_region_centroids) {
    float min_squared_distance = kMaxSquaredDistance;
    int min_distance_index = -1;
    for (int j = 0; j < points.Size(); ++j) {
      const auto& point = points.AtUnsafe(j);
      const auto& normal = normals.AtUnsafe(j);
      float squared_distance = (seed_point.point - point).squaredNorm();
      if (squared_distance < min_squared_distance &&
          seed_point.normal.dot(normal) > kCosMaxNormalDifferenceAngle) {
        min_squared_distance = squared_distance;
        min_distance_index = j;
      }
    }

    if (min_distance_index >= 0) {
      seed_point_indices->push_back(min_distance_index);
    }
  }
}

}  // namespace mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANAR_REGION_H_
