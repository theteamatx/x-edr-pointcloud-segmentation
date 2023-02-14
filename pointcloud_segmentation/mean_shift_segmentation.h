#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_MEAN_SHIFT_SEGMENTATION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_MEAN_SHIFT_SEGMENTATION_H_

#include <queue>
#include <vector>

#include "googlex/proxy/eigenmath/types.h"
#include "googlex/proxy/eigenmath/utils.h"
#include "googlex/proxy/object_properties/point_cloud/cluster_region.h"
#include "googlex/proxy/object_properties/point_cloud/region_segmentation_config.proto.h"
#include "third_party/eigen3/Eigen/Core"

namespace blue::mobility {

namespace internal {

// If the distance between a point and a object centroid within this threshold,
// it will contribute to neigbouring shift.
constexpr double kSquareDistanceThreshold = 1;

// A search window for meanshift compute box. A larger search window will
// require less iteration but takes longer time for each iteration. A very small
// window will track the seed point at local miminum. The maximum shift of the
// seed per iteration is bounded by this value.
constexpr int kHalfSearchWindow = 5;

// A mean-shift seed is considered as valid if its intensity exceeds the
// threshold.
constexpr double kIntensityRatioThreshold = 0.5;

// If the squred distance between a point and a object centroid within this
// threshold, it will considered as an inlier.
constexpr double kSquaredCentroidDistanceThreshold = 1.0;

// If the squred distance between a point and a inlier within this threshold, it
// will considered as an inlier.
constexpr double kSquaredNeighorDistanceThreshold = 0.2 * 0.2;

// A structure represent a 2D index in double format.
class PointIndex : public eigenmath::Vector2f {
 public:
  PointIndex(const eigenmath::Vector2f& other) : eigenmath::Vector2f(other) {}
  PointIndex(const PointIndex& other)
      : eigenmath::Vector2f(static_cast<const eigenmath::Vector2f&>(other)) {}
  PointIndex(eigenmath::Vector2f&& other) : eigenmath::Vector2f(other) {}
  PointIndex(PointIndex&& other)
      : eigenmath::Vector2f(static_cast<eigenmath::Vector2f&&>(other)) {}

  PointIndex& operator=(const PointIndex& other) {
    if (this != &other) {
      x() = other.x();
      y() = other.y();
    }
    return *this;
  }

  PointIndex(int row_in, int col_in);
  PointIndex(float row_in, float col_in);
  float Row() const { return x(); }
  float Col() const { return y(); }
  int RowIndex() const;
  int ColIndex() const;
};

// A structure represent a seed point location and associated index in
// pointcloud.
template <typename PointType>
struct SeedPointAndIndex {
  PointType seed_point;
  PointIndex index;
};

// TODO(b/138958112) Add Gaussian Kernel.
// One type of kernel for the seed point weighting.
template <typename PointType>
const eigenmath::VectorXd FlatKernelWeights(
    const std::vector<SeedPointAndIndex<PointType>>& neighbors_shift) {
  return eigenmath::VectorXd::Ones(neighbors_shift.size());
}

// Search neighbors of the centroid. For neighbors within the distance
// threshold, shift of distances and indices are return.
template <typename PointType>
std::vector<SeedPointAndIndex<PointType>> GetNeighborsShift(
    const Cloud<PointType>& point_cloud,
    const SeedPointAndIndex<PointType>& centroid, const Cloud<int>& labels,
    double square_distance_threshold, int half_search_window) {
  std::vector<SeedPointAndIndex<PointType>> neighbors_shift;

  const int row = centroid.index.RowIndex();
  const int col = centroid.index.ColIndex();
  for (int delta_col = -half_search_window; delta_col <= half_search_window;
       ++delta_col) {
    for (int delta_row = -half_search_window; delta_row <= half_search_window;
         ++delta_row) {
      const int col_current = col + delta_col;
      const int row_current = row + delta_row;
      if (col_current < 0 || col_current >= point_cloud.Cols() ||
          row_current < 0 || row_current >= point_cloud.Rows() ||
          labels.At(row_current, col_current) != kUnlabeled ||
          point_cloud.At(row_current, col_current).hasNaN()) {
        continue;
      }

      const SeedPointAndIndex<PointType> shift = {
          point_cloud.At(row_current, col_current) - centroid.seed_point,
          PointIndex(PointIndex(row_current, col_current) - centroid.index)};

      const PointType& distance = shift.seed_point;
      if (distance.squaredNorm() > square_distance_threshold) {
        continue;
      }
      neighbors_shift.emplace_back(shift);
    }
  }
  return neighbors_shift;
}

// Applied kernel weighting on the neighbor shifts. Return the weight shift.
template <typename PointType>
SeedPointAndIndex<PointType> GetWeightedShift(
    const eigenmath::VectorXd& kernel_weights,
    const std::vector<SeedPointAndIndex<PointType>>& neighbors_shift) {
  CHECK_EQ(kernel_weights.size(), neighbors_shift.size());
  PointType shift_distance = PointType::Zero();
  PointIndex shift_index(0, 0);
  for (int i = 0; i < kernel_weights.size(); ++i) {
    shift_distance += kernel_weights[i] * neighbors_shift.at(i).seed_point;
    shift_index += kernel_weights[i] * neighbors_shift.at(i).index;
  }
  return {shift_distance, shift_index};
}

// Determine whether neighbor points near given center are inliers of this
// cluster region. Points are considered inliers based on their distance to
// seed/centroid of the cluster and distance to neigbor center. When
// first_unexamined_index is set to true, distance to neigbor center is ignored.
template <typename PointType>
void AddNeighborsAndGrowReigon(const Cloud<PointType>& point_cloud,
                               int neighbor_center_index,
                               double squared_centroid_distance_threshold,
                               double squared_neighor_distance_threshold,
                               bool first_unexamined_index, Cloud<int>* labels,
                               ClusterRegion<>* cluster_regoin,
                               std::queue<int>* unexamined_seed_indices) {
  CHECK_NE(unexamined_seed_indices, nullptr);
  const int half_search_window = cluster_regoin->config().half_search_window();
  int row, col;
  point_cloud.IndexToRowCol(neighbor_center_index, &row, &col);
  for (int delta_col = -half_search_window; delta_col <= half_seach_window;
       ++delta_col) {
    for (int delta_row = -half_search_window; delta_row <= half_seach_window;
         ++delta_row) {
      if (!first_unexamined_index && delta_col == 0 && delta_row == 0) {
        continue;
      }
      const int col_current = col + delta_col;
      const int row_current = row + delta_row;
      if (col_current < 0 || col_current >= point_cloud.Cols() ||
          row_current < 0 || row_current >= point_cloud.Rows() ||
          labels->AtUnsafe(row_current, col_current) != kUnlabeled ||
          point_cloud.At(row_current, col_current).hasNaN()) {
        continue;
      }
      const auto& candidate_point =
          point_cloud.AtUnsafe(row_current, col_current);

      // A neigbor is consider as an inlier if it is within centroid distance
      // threshold or connected to the seed point.
      CHECK(cluster_regoin->Seed().has_value());
      if ((candidate_point - cluster_regoin->Seed().value()).squaredNorm() >
          squared_centroid_distance_threshold) {
        if (first_unexamined_index ||
            (candidate_point - point_cloud.AtUnsafe(neighbor_center_index))
                    .squaredNorm() > squared_neighor_distance_threshold) {
          continue;
        }
      }
      const int inlier_index = labels->LinearizeIndex(row_current, col_current);
      cluster_regoin->mutable_inlier_indices()->push_back(inlier_index);
      labels->AtUnsafe(inlier_index) = cluster_regoin->label_id();
      unexamined_seed_indices->push(inlier_index);
    }
  }
}

}  // namespace internal

// Given a point cloud, compute the mean shifted clusters and update the result
// in segmentation labels. This method uses the neighbor relationship of the
// point cloud and applies a sliding neighbor window for shift direction
// computation.
template <typename PointType>
void SlidingMeanShift(const Cloud<PointType>& point_cloud,
                      const ClusterRegionConfigProto& config, int iterations,
                      int initial_region_id_offset, Cloud<int>* labels,
                      std::vector<ClusterRegion<>>* regions) {
  CHECK_NE(labels, nullptr);
  CHECK_NE(regions, nullptr);
  CHECK_EQ(point_cloud.Cols(), labels->Cols());
  CHECK_EQ(point_cloud.Rows(), labels->Rows());
  regions->clear();
  std::vector<double> intensities(point_cloud.size(), 1.0);
  // Copy the unlabeled point of the original point cloud as seed_point.
  std::vector<internal::SeedPointAndIndex<PointType>> seed_points_and_indices;
  for (int index = 0; index < labels->size(); ++index) {
    if (labels->At(index) == kUnlabeled &&
        !point_cloud.AtUnsafe(index).hasNaN()) {
      int row, col;
      point_cloud.IndexToRowCol(index, &row, &col);
      seed_points_and_indices.emplace_back(
          internal::SeedPointAndIndex<PointType>(
              {point_cloud.AtUnsafe(index), internal::PointIndex(row, col)}));
    }
  }
  std::vector<bool> valid_seed_point(seed_points_and_indices.size(), true);
  // Compute mean-shift for each point.
  for (int iteration = 0; iteration < iterations; ++iteration) {
    for (int i = 0; i < seed_points_and_indices.size(); ++i) {
      if (!valid_seed_point.at(i)) {
        continue;
      }

      const auto neighbors_shift = internal::GetNeighborsShift(
          point_cloud, seed_points_and_indices.at(i), *labels,
          internal::kSquareDistanceThreshold, internal::kHalfSearchWindow);

      const eigenmath::VectorXd kernel_weights =
          FlatKernelWeights(neighbors_shift);
      const double total_weight = kernel_weights.sum();
      if (total_weight < internal::kIntensityRatioThreshold *
                             internal::kHalfSearchWindow *
                             internal::kHalfSearchWindow * 4) {
        valid_seed_point.at(i) = false;
        continue;
      }
      // For each seed point, compute the shift direction.
      const internal::SeedPointAndIndex<PointType> weighted_shift =
          GetWeightedShift(kernel_weights, neighbors_shift);
      seed_points_and_indices.at(i).seed_point +=
          weighted_shift.seed_point / total_weight;
      seed_points_and_indices.at(i).index +=
          weighted_shift.index / total_weight;
      intensities.at(i) = total_weight;
    }
  }

  // Post-processing: remove near duplicate points. If the distance between two
  // kernels is less than the bandwidth, then we have to remove one because it
  // is a duplicate. Remove the one with lower intensity.

  // Sort points by their intensity.
  std::vector<std::pair<internal::SeedPointAndIndex<PointType>, double>>
      seed_points_and_intensities;
  seed_points_and_intensities.reserve(seed_points_and_indices.size());
  for (int i = 0; i < seed_points_and_indices.size(); i++) {
    if (valid_seed_point.at(i)) {
      seed_points_and_intensities.push_back(
          std::make_pair(seed_points_and_indices.at(i), intensities[i]));
    }
  }
  std::sort(
      seed_points_and_intensities.begin(), seed_points_and_intensities.end(),
      [](const std::pair<internal::SeedPointAndIndex<PointType>, double>& left,
         const std::pair<internal::SeedPointAndIndex<PointType>, double>&
             right) { return left.second < right.second; });

  // Combine points into clusters by their neighborhood.
  std::vector<bool> valid_sorted_seeds(seed_points_and_intensities.size(),
                                       true);
  for (int i = 0; i < seed_points_and_intensities.size(); i++) {
    if (valid_sorted_seeds.at(i)) {
      ClusterRegion<PointType> cluster_regoin(config);
      *cluster_regoin.mutable_label_id() =
          regions->size() + initial_region_id_offset;
      const auto& [centroid_and_index, intensity] =
          seed_points_and_intensities.at(i);
      cluster_regoin.SetSeed(
          static_cast<eigenmath::Vector3f>(centroid_and_index.seed_point));
      std::queue<int> unexamined_seed_indices;
      unexamined_seed_indices.emplace(
          point_cloud.LinearizeIndex(centroid_and_index.index.RowIndex(),
                                     centroid_and_index.index.ColIndex()));
      bool first_unexamined_index = true;
      while (!unexamined_seed_indices.empty()) {
        const int testing_index = unexamined_seed_indices.front();
        unexamined_seed_indices.pop();
        internal::AddNeighborsAndGrowReigon(
            point_cloud, testing_index,
            internal::kSquaredCentroidDistanceThreshold,
            internal::kSquaredNeighorDistanceThreshold, first_unexamined_index,
            labels, &cluster_regoin, &unexamined_seed_indices);
        first_unexamined_index = false;
      }

      // Finalized the regoin if it is a valid regoin, mark the other seed
      // points in neigbour range as invalid.
      if (cluster_regoin.CheckAndFinalize(point_cloud, *labels)) {
        for (int j = i + 1; j < seed_points_and_intensities.size(); j++) {
          if (valid_sorted_seeds.at(j) &&
              (seed_points_and_intensities.at(j).first.seed_point -
               centroid_and_index.seed_point)
                      .squaredNorm() <
                  internal::kSquaredCentroidDistanceThreshold) {
            valid_sorted_seeds.at(j) = false;
          }
        }
        regions->push_back(cluster_regoin);
      } else {
        for (const int i : cluster_regoin.inlier_indices()) {
          labels->AtUnsafe(i) = kUnlabeled;
        }
      }
    }
  }
}

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_MEAN_SHIFT_SEGMENTATION_H_
