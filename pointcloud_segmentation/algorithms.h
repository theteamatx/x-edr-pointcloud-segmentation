// Copyright 2023 Google LLC

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_ALGORITHMS_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_ALGORITHMS_H_

#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "eigenmath/plane_conversions.h"
#include "eigenmath/pose3.h"
#include "pointcloud_segmentation/indices.h"
#include "pointcloud_segmentation/plane_estimator.h"
#include "collision/convex_hull.h"

namespace blue::mobility {
namespace detail {

// Tests if 3D point p is inside the triangle (u, v, w). Works for both
// clockwise and counterclockwise triangles
template <typename Derived>
bool IsInsideTriangle(const Eigen::MatrixBase<Derived>& u,
                      const Eigen::MatrixBase<Derived>& v,
                      const Eigen::MatrixBase<Derived>& w,
                      const Eigen::MatrixBase<Derived>& p) {
  using Scalar = typename Derived::Scalar;
  // see http://www.blackpawn.com/texts/pointinpoly/default.html
  // Barycentric Coordinates
  const Eigen::Matrix<Scalar, 3, 1> v0 = w - u;
  const Eigen::Matrix<Scalar, 3, 1> v1 = v - u;
  const Eigen::Matrix<Scalar, 3, 1> v2 = p - u;

  // Compute dot products
  const Scalar dot00 = v0.dot(v0);
  const Scalar dot01 = v0.dot(v1);
  const Scalar dot02 = v0.dot(v2);
  const Scalar dot11 = v1.dot(v1);
  const Scalar dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  const Scalar invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  const Scalar a = (dot11 * dot02 - dot01 * dot12) * invDenom;
  const Scalar b = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (a >= 0) && (b >= 0) && (a + b < 1);
}

// Tests if the triangle defined by the indices (u, v, w) is an ear of the
// given polygon (defined by the indices in polygon_indices). For this,
// (u, v, w) needs to define a convex triangle of the clockwise polygon and
// cannot contain any other polygon vertices. Index v is always the ear center.
template <typename PointT>
bool IsEar(const Cloud<PointT>& points, int u, int v, int w,
           const std::vector<int>& polygon_indices,
           const Eigen::Vector3f& polygon_normal) {
  const auto& point_u = points.AtUnsafe(polygon_indices[u]);
  const auto& point_v = points.AtUnsafe(polygon_indices[v]);
  const auto& point_w = points.AtUnsafe(polygon_indices[w]);

  constexpr float kEps = 1e-25f;

  const Eigen::Vector3f triangle_normal(
      (point_v - point_u).cross(point_w - point_u));
  // Avoid flat and concave triangles.
  if (triangle_normal.squaredNorm() < kEps ||
      polygon_normal.dot(triangle_normal) < 0.f) {
    return false;
  }

  // Check if any other vertex is inside the triangle.
  for (int k = 0; k < static_cast<int>(polygon_indices.size()); ++k) {
    if ((k == u) || (k == v) || (k == w)) {
      continue;
    }
    const auto& point = points.AtUnsafe(polygon_indices[k]);

    if (IsInsideTriangle(point_u, point_v, point_w, point)) {
      return false;
    }
  }
  return true;
}

// Creates an index vector with a minimal neighborhood of the closest 4 points
// with a distance greater than minimum_neighbor_distance plus the center point.
// If include_diagonal_neighbors is true, the 8-neighborhood is considered
template <typename PointT>
int FindNormalSupportNeighbors(const Cloud<PointT>& point_cloud, int col,
                               int row, float min_neighbor_distance,
                               float max_neighbor_distance,
                               bool include_diagonal_neighbors,
                               std::vector<int>* neighbor_indices) {
  const float kMinNeighborDistanceSquared =
      min_neighbor_distance * min_neighbor_distance;
  const float kMaxNeighborDistanceSquared =
      max_neighbor_distance * max_neighbor_distance;

  neighbor_indices->clear();
  if (include_diagonal_neighbors) {
    neighbor_indices->reserve(9);
  } else {
    neighbor_indices->reserve(5);
  }

  const auto& center = point_cloud.AtUnsafe(row, col);

  if (!center.allFinite()) {
    return 0;
  }

  int min_row = std::max(row - 1, 0);
  int max_row = std::min(row + 1, point_cloud.Rows() - 1);
  int min_col = std::max(col - 1, 0);
  int max_col = std::min(col + 1, point_cloud.Cols() - 1);

  neighbor_indices->push_back(point_cloud.LinearizeIndex(row, col));

  // upper neighbor
  int index = point_cloud.LinearizeIndex(row - 1, col);
  for (int r = row - 1; r >= 0; --r, index -= 1) {
    const auto& p = point_cloud.AtUnsafe(index);
    if (!p.allFinite()) {
      continue;
    }
    const float squared_distance = (center - p).squaredNorm();
    if (squared_distance >= kMinNeighborDistanceSquared &&
        squared_distance <= kMaxNeighborDistanceSquared) {
      neighbor_indices->push_back(index);
      min_row = r;
      break;
    }
  }

  // left neighbor
  index = point_cloud.LinearizeIndex(row, col - 1);
  for (int c = col - 1; c >= 0; --c, index -= point_cloud.Rows()) {
    const auto& p = point_cloud.AtUnsafe(index);
    if (!p.allFinite()) {
      continue;
    }
    const float squared_distance = (center - p).squaredNorm();
    if (squared_distance >= kMinNeighborDistanceSquared &&
        squared_distance <= kMaxNeighborDistanceSquared) {
      neighbor_indices->push_back(index);
      min_col = c;
      break;
    }
  }

  // lower neighbor
  index = point_cloud.LinearizeIndex(row + 1, col);
  for (int r = row + 1; r < point_cloud.Rows(); ++r, index += 1) {
    const auto& p = point_cloud.AtUnsafe(index);
    if (!p.allFinite()) {
      continue;
    }
    const float squared_distance = (center - p).squaredNorm();
    if (squared_distance >= kMinNeighborDistanceSquared &&
        squared_distance <= kMaxNeighborDistanceSquared) {
      neighbor_indices->push_back(index);
      max_row = r;
      break;
    }
  }

  // right neighbor
  index = point_cloud.LinearizeIndex(row, col + 1);
  for (int c = col + 1; c < point_cloud.Cols();
       ++c, index += point_cloud.Rows()) {
    const auto& p = point_cloud.AtUnsafe(index);
    if (!p.allFinite()) {
      continue;
    }
    const float squared_distance = (center - p).squaredNorm();
    if (squared_distance >= kMinNeighborDistanceSquared &&
        squared_distance <= kMaxNeighborDistanceSquared) {
      neighbor_indices->push_back(index);
      max_col = c;
      break;
    }
  }

  // Return early if diagonals are not required.
  if (!include_diagonal_neighbors) {
    return neighbor_indices->size();
  }

  // left diagonals
  if (min_col != col) {
    // upper-left neighbor
    if (min_row != row) {
      index = point_cloud.LinearizeIndex(min_row, min_col);
      const auto& p = point_cloud.AtUnsafe(index);
      const float squared_distance = (center - p).squaredNorm();
      if (p.allFinite() && squared_distance >= kMinNeighborDistanceSquared &&
          squared_distance <= kMaxNeighborDistanceSquared) {
        neighbor_indices->push_back(index);
      }
    }

    // lower-left neighbor
    if (max_row != row) {
      index = point_cloud.LinearizeIndex(max_row, min_col);
      const auto& p = point_cloud.AtUnsafe(index);
      const float squared_distance = (center - p).squaredNorm();
      if (p.allFinite() && squared_distance >= kMinNeighborDistanceSquared &&
          squared_distance <= kMaxNeighborDistanceSquared) {
        neighbor_indices->push_back(index);
      }
    }
  }

  // right diagonals
  if (max_col != col) {
    // upper-right neighbor
    if (min_row != row) {
      index = point_cloud.LinearizeIndex(min_row, max_col);
      const auto& p = point_cloud.AtUnsafe(index);
      const float squared_distance = (center - p).squaredNorm();
      if (p.allFinite() && squared_distance >= kMinNeighborDistanceSquared &&
          squared_distance <= kMaxNeighborDistanceSquared) {
        neighbor_indices->push_back(index);
      }
    }

    // lower-right neighbor
    if (max_row != row) {
      index = point_cloud.LinearizeIndex(max_row, max_col);
      const auto& p = point_cloud.AtUnsafe(index);
      const float squared_distance = (center - p).squaredNorm();
      if (p.allFinite() && squared_distance >= kMinNeighborDistanceSquared &&
          squared_distance <= kMaxNeighborDistanceSquared) {
        neighbor_indices->push_back(index);
      }
    }
  }

  return neighbor_indices->size();
}

}  // namespace detail

// Concave polygon normal using Stoke's law:
// https://en.wikipedia.org/wiki/Stokes%27_theorem
// The resulting vector has the direction of the normal but not unit length
template <typename PointT, typename IndicesT = NoIndices>
Eigen::Vector3f CumulativePolygonNormal(
    const Cloud<PointT>& points, const IndicesT& polygon_indices = IndicesT()) {
  Eigen::Vector3f normal(0.0f, 0.0f, 0.0f);
  int size = GetIndicesSize(points, polygon_indices);
  for (int i = 0; i < size; ++i) {
    normal +=
        points.AtUnsafe(GetIndex(i, polygon_indices))
            .cross(points.AtUnsafe(GetIndex((i + 1) % size, polygon_indices)));
  }
  return normal;
}

// Concave polygon normal using Stoke's law:
// https://en.wikipedia.org/wiki/Stokes%27_theorem
template <typename PointT, typename IndicesT = NoIndices>
Eigen::Vector3f PolygonNormal(const Cloud<PointT>& points,
                              const IndicesT& polygon_indices = IndicesT()) {
  Eigen::Vector3f normal = CumulativePolygonNormal(points, polygon_indices);
  return normal / normal.norm();
}

// Concave polygon area using Stoke's law:
// https://en.wikipedia.org/wiki/Stokes%27_theorem
template <typename PointT, typename IndicesT = NoIndices>
float PolygonArea(const Cloud<PointT>& points,
                  const IndicesT& polygon_indices = IndicesT()) {
  return 0.5f * CumulativePolygonNormal(points, polygon_indices).norm();
}

template <typename PointT, typename IndicesT = NoIndices>
bool IsPolygonAreaZero(const Cloud<PointT>& points,
                       const IndicesT& polygon_indices = IndicesT(),
                       const float eps = 1.0e-7f) {
  return CumulativePolygonNormal(points, polygon_indices).squaredNorm() < eps;
}

template <typename PointT, typename IndicesT>
void ProjectPointsToPlane(const Cloud<PointT>& points, const IndicesT& indices,
                          const eigenmath::Plane3f& plane,
                          Cloud<PointT>* projected_points) {
  projected_points->resize(GetIndicesSize(points, indices));
  for (int i = 0; i < GetIndicesSize(points, indices); ++i) {
    (*projected_points)[i] = points[GetIndex(i, indices)];
    (*projected_points).AtUnsafe(i) =
        plane.projection((*projected_points).AtUnsafe(i));
  }
}

struct ComputeNormalsParams {
  // Minimum distance a neighbor can have to be chosen for normal computation
  float min_neighbor_distance = 0.1;  // meters
  // Maximum distance a neighbor can have to be chosen for normal computation
  float max_neighbor_distance = 1.0;  // meters
  // Expands the neighbor candidates to the 8-neighborhood
  bool include_diagonal_neighbors = true;
  // Minimum number of support neighbors required to compute a valid normal
  int min_num_support_neighbors = 4;
};

// Computes normals from an organized point cloud using a minimal neighborhood
// of the closest 4 points (or 8 if include_diagonal_neighbors is true) with a
// distance greater than minimum_neighbor_distance plus the center point. Choose
// the minimum distance large enough so that you get stable non-noisy normals,
// but small enough to capture the kind of detail you are interested in.
template <typename PointT, typename NormalT>
void ComputeNormalsOrganized(
    const eigenmath::Pose3d& point_cloud_pose_sensor,
    const Cloud<PointT>& points, Cloud<NormalT>* normals,
    const std::pair<int, int>& row_range, const std::pair<int, int>& col_range,
    const ComputeNormalsParams& params = ComputeNormalsParams()) {
  constexpr float kInvalidValue = std::numeric_limits<float>::quiet_NaN();

  PlaneEstimator plane_estimator;
  std::vector<int> neighbor_indices;

  // Iterating over the entire index vector
  for (int col = col_range.first; col < col_range.second; ++col) {
    for (int row = row_range.first; row < row_range.second; ++row) {
      const auto& point = points.AtUnsafe(row, col);
      auto& normal = normals->AtUnsafe(row, col);
      if (!point.allFinite() ||
          detail::FindNormalSupportNeighbors(
              points, col, row, params.min_neighbor_distance,
              params.max_neighbor_distance, params.include_diagonal_neighbors,
              &neighbor_indices) < params.min_num_support_neighbors) {
        normal = eigenmath::Vector3f::Constant(kInvalidValue);
        continue;
      }

      plane_estimator.SetNormalOrientation(
          point_cloud_pose_sensor.translation().cast<float>() - point);
      plane_estimator.AddPoints(points, neighbor_indices);

      if (!plane_estimator.PlaneValid()) {
        normal = eigenmath::Vector3f::Constant(kInvalidValue);
        continue;
      }
      normal = plane_estimator.Plane().normal();
      plane_estimator.Clear();
    }
  }
}

template <typename PointT, typename NormalT>
void ComputeNormalsOrganized(
    const eigenmath::Pose3d& point_cloud_pose_sensor,
    const Cloud<PointT>& points, Cloud<NormalT>* normals,
    const ComputeNormalsParams& params = ComputeNormalsParams()) {
  ComputeNormalsOrganized(point_cloud_pose_sensor, points, normals,
                          {0, points.Rows()},
                          {0, points.Cols()}, params);
}

// Ear-clipping triangulation for concave polygons. The output triangles have
// size num_triangles * 3. If add_to_triangles is true, the generated triangles
// are added instead of first clearing triangles. This triangulation handles
// "simple" concave polygons (i.e. no holes or self intersections are allowed
// and any given index in polygon_indices can only appear once). If the input
// polygon_indices define a clockwise polygon, also all resulting triangles are
// clockwise. Likewise, a counterclockwise polygon results in counterclockwise
// triangles
template <typename PointT>
bool TriangulatePolygon(const Cloud<PointT>& points,
                        std::vector<int> polygon_indices,
                        std::vector<int>* triangles,
                        bool add_to_triangles = false) {
  if (!add_to_triangles) {
    triangles->clear();
  }

  if (polygon_indices.size() < 3) {
    return false;
  }
  triangles->reserve(triangles->size() + 3 * (polygon_indices.size() - 2));

  // Avoid closed loops.
  if (polygon_indices.front() == polygon_indices.back()) {
    polygon_indices.erase(polygon_indices.end() - 1);
    if (polygon_indices.size() < 3) {
      return false;
    }
  }

  const Eigen::Vector3f polygon_normal =
      CumulativePolygonNormal(points, polygon_indices);

  // null_iterations avoids infinite loops if the polygon is not simple.
  for (int u = static_cast<int>(polygon_indices.size()) - 1,
           null_iterations = 0;
       polygon_indices.size() > 2 &&
       null_iterations < polygon_indices.size() * 3;
       ++null_iterations, u = (u + 1) % polygon_indices.size()) {
    const int v = (u + 1) % polygon_indices.size();
    const int w = (u + 2) % polygon_indices.size();

    if (detail::IsEar(points, u, v, w, polygon_indices, polygon_normal)) {
      triangles->push_back(polygon_indices[u]);
      triangles->push_back(polygon_indices[v]);
      triangles->push_back(polygon_indices[w]);
      polygon_indices.erase(polygon_indices.begin() + v);
      null_iterations = 0;
    }
  }
  return polygon_indices.size() == 2;
}

template <typename PointT>
bool TriangulatePolygonRecursive(const Cloud<PointT>& points,
                                 std::vector<int> polygon_indices,
                                 const Eigen::Vector3f& polygon_normal,
                                 std::vector<int>* triangles) {
  // If the polygon area is zero we do not need to generate flat triangles
  if (IsPolygonAreaZero(points, polygon_indices)) {
    return true;
  }

  // Find index duplicates (i.e. loops)
  int start_loop_idx = -1, end_loop_idx = -1;
  for (int i = 0; i < polygon_indices.size(); ++i) {
    for (int j = i + 1; j < polygon_indices.size(); ++j) {
      if (polygon_indices[i] == polygon_indices[j]) {
        start_loop_idx = i;
        end_loop_idx = j;
        break;
      }
    }
  }

  // Did we find a loop? If so, split polygon_indices into the loopy part and
  // the remainder and continue recursing
  if (start_loop_idx >= 0) {
    std::vector<int> loop_indices(polygon_indices.begin() + start_loop_idx,
                                  polygon_indices.begin() + end_loop_idx);
    polygon_indices.erase(polygon_indices.begin() + start_loop_idx,
                          polygon_indices.begin() + end_loop_idx);
    return TriangulatePolygonRecursive(points, std::move(loop_indices),
                                       polygon_normal, triangles) &&
           TriangulatePolygonRecursive(points, std::move(polygon_indices),
                                       polygon_normal, triangles);
  } else {
    // Do the actual ear-clipping triangulation here.
    // null_iterations avoids infinite loops if the polygon is not simple.
    for (int u = static_cast<int>(polygon_indices.size()) - 1,
             null_iterations = 0;
         polygon_indices.size() > 2 &&
         null_iterations < polygon_indices.size() * 2;
         ++null_iterations, u = (u + 1) % polygon_indices.size()) {
      const int v = (u + 1) % polygon_indices.size();
      const int w = (u + 2) % polygon_indices.size();

      if (detail::IsEar(points, u, v, w, polygon_indices, polygon_normal)) {
        triangles->push_back(polygon_indices[u]);
        triangles->push_back(polygon_indices[v]);
        triangles->push_back(polygon_indices[w]);
        polygon_indices.erase(polygon_indices.begin() + v);
        null_iterations = 0;
      }
    }
    return polygon_indices.size() == 2;
  }
}

// Ear-clipping triangulation for concave polygons. The output triangles have
// size num_triangles * 3. If add_to_triangles is true, the generated triangles
// are added instead of first clearing triangles. This triangulation handles
// "simple" concave polygons (i.e. no holes or self intersections are allowed).
// Duplicate indices are allowed, but flat triangles are omitted in the output.
// If the input polygon_indices define a clockwise polygon, also all resulting
// triangles are clockwise. Likewise, a counterclockwise polygon results in
// counterclockwise triangles
template <typename PointT>
bool TriangulateLoopyPolygon(const Cloud<PointT>& points,
                             std::vector<int> polygon_indices,
                             std::vector<int>* triangles,
                             bool add_to_triangles = false) {
  if (!add_to_triangles) {
    triangles->clear();
  }

  if (polygon_indices.size() < 3) {
    return false;
  }
  triangles->reserve(triangles->size() + 3 * (polygon_indices.size() - 2));

  // Avoid closed loops.
  if (polygon_indices.front() == polygon_indices.back()) {
    polygon_indices.erase(polygon_indices.end() - 1);
    if (polygon_indices.size() < 3) {
      return false;
    }
  }

  const Eigen::Vector3f polygon_normal =
      CumulativePolygonNormal(points, polygon_indices);

  return TriangulatePolygonRecursive(points, std::move(polygon_indices),
                                     polygon_normal, triangles);
}

// Computes the convex hull for the points specified by indices in the given
// plane. The output convex_hull_points lie perfectly in the plane, even if the
// input points don't.
template <typename PointT1, typename PointT2, typename IndicesT>
void PlanarConvexHull(const Cloud<PointT1>& points, const IndicesT& indices,
                      const eigenmath::Plane3f& plane,
                      CloudBuffer<PointT2>* convex_hull_points) {
  eigenmath::Pose3f world_pose_plane = eigenmath::PoseFromPlane(plane);
  eigenmath::Pose3f plane_pose_world = world_pose_plane.inverse();
  std::vector<eigenmath::Vector2d> plane_points(
      GetIndicesSize(points, indices));
  for (int i = 0; i < GetIndicesSize(points, indices); ++i) {
    const auto& boundary_point = points.AtUnsafe(GetIndex(i, indices));
    eigenmath::Vector3f plane_point = plane_pose_world * boundary_point;
    plane_points[i] = {plane_point.x(), plane_point.y()};
  }

  ::mobility::collision::ConvexHull convex_hull(plane_points);
  std::vector<eigenmath::Vector2d> convex_plane_points =
      convex_hull.GetPoints();
  for (int i = 0; i < convex_plane_points.size(); ++i) {
    PointT2& convex_hull_point = convex_hull_points->AtUnsafe(i);
    eigenmath::Vector3f plane_point(convex_plane_points[i].x(),
                                    convex_plane_points[i].y(), 0.0f);
    convex_hull_point = world_pose_plane * plane_point;
  }
}

// Compute the best-fit transform between two point clouds of the same size with
// corresponded points.
template <typename PointT, typename Scalar>
bool ComputeRigidTransform(const Cloud<PointT>& source,
                           const Cloud<PointT>& dest,
                           eigenmath::Pose3<Scalar>* dest_pose_source) {
  CHECK_EQ(source.size(), dest.size());
  const int num_points = source.size();

  // Compute the centroids of the two point clouds. The optimal translation is
  // then c_d - c_s;
  const eigenmath::Vector3<Scalar> c_s =
      std::accumulate(source.begin(), source.end(),
                      eigenmath::Vector3<Scalar>(0.0, 0.0, 0.0)) /
      num_points;
  const eigenmath::Vector3<Scalar> c_d =
      std::accumulate(dest.begin(), dest.end(),
                      eigenmath::Vector3<Scalar>(0.0, 0.0, 0.0)) /
      num_points;

  // To compute the optimal rotation, form two matrices S and D with columns of
  // S containing the vectors xi - c_s from the centroid of S, and similarly,
  // with the columns of D being xi - c_d. The optimal rotation R satisfies:
  //
  // R = argmin_{M \in O(3)} ||RS - D||^2.
  eigenmath::Matrix3<Scalar> SDt = eigenmath::Matrix3<Scalar>::Zero();
  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3<Scalar> s = source.AtUnsafe(i) - c_s;
    const eigenmath::Vector3<Scalar> d = dest.AtUnsafe(i) - c_d;
    SDt += s * d.transpose();
  }

  // It can be shown that the solution to this optimization problem is R = UV^T
  // where U and V are the left and right singular vector matrices of the
  // singular value decomposition of SD^T.
  Eigen::JacobiSVD<eigenmath::Matrix3<Scalar>, Eigen::NoQRPreconditioner> svd{
      SDt, Eigen::ComputeFullU | Eigen::ComputeFullV};
  if (!svd.computeU() || !svd.computeV()) {
    return false;
  }

  // Catch the case where det(R_mat) = -1, i.e., it would mirror the point
  // cloud.
  eigenmath::Matrix3<Scalar> S = eigenmath::Matrix3<Scalar>::Identity();
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) {
    S(2, 2) = -1;
  }

  // Compute transform.
  const eigenmath::Matrix3<Scalar> R_mat =
      svd.matrixV() * (svd.matrixU() * S).transpose();
  const eigenmath::Quaternion<Scalar> R =
      eigenmath::Quaternion<Scalar>(R_mat).normalized();
  dest_pose_source->setQuaternion(R);
  dest_pose_source->translation() = (c_d - R_mat * c_s);
  return true;
}

// Find the indices in destination point cloud that are closest to each point in
// the source point cloud.
template <typename PointT>
void NearestNeighbors(const Cloud<PointT>& source, const Cloud<PointT>& dest,
                      std::vector<int>* indices) {
  indices->resize(source.size(), 0);
  for (int source_col = 0; source_col < source.size(); source_col++) {
    const PointT& source_point = source.AtUnsafe(source_col);
    int closest_point = 0;
    double closest_distance =
        (source_point - dest.AtUnsafe(closest_point)).squaredNorm();
    for (int dest_col = 1; dest_col < dest.size(); dest_col++) {
      const PointT& dest_point = dest.AtUnsafe(dest_col);
      const double distance = (source_point - dest_point).squaredNorm();
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_point = dest_col;
      }
    }
    (*indices)[source_col] = closest_point;
  }
  CHECK(indices->size() == source.size());
}

// Finds the best-fit transform that maps a source point cloud to a destination
// point cloud. Makes no assumptions on correspondence between points or the
// size of the source and destination point clouds.
template <typename PointT>
double IterativeClosestPoint(const Cloud<PointT>& source,
                             const Cloud<PointT>& dest,
                             eigenmath::Pose3f* dest_pose_source,
                             int max_iterations = 20, double tolerance = 1e-3) {
  CloudBuffer<PointT> transformed_source =
      const_cast<Cloud<PointT>&>(source).Copy();
  CloudBuffer<PointT> neighbor_cloud(transformed_source.size());

  double prev_error = 0.0;
  double mean_error = 0.0;
  std::vector<int> nearest_neighbors;
  *dest_pose_source = eigenmath::Pose3f::Identity();
  for (int iter = 0; iter < max_iterations; iter++) {
    // For each point in the transformed source point cloud, match the closest
    // point in the reference point cloud.
    NearestNeighbors(transformed_source, dest, &nearest_neighbors);
    for (int col = 0; col < transformed_source.size(); col++) {
      neighbor_cloud.AtUnsafe(col) = dest.AtUnsafe(nearest_neighbors[col]);
    }

    // Estimate the combination of rotation and translation using a root mean
    // square point to point distance metric minimizer.
    eigenmath::Pose3f dest_pose_transformed_source;
    CHECK(ComputeRigidTransform(transformed_source, neighbor_cloud,
                                     &dest_pose_transformed_source));

    // Transform the source points using the obtained transformation.
    mean_error = 0.0;
    for (int index = 0; index < transformed_source.size(); index++) {
      transformed_source.AtUnsafe(index) =
          dest_pose_transformed_source * transformed_source.AtUnsafe(index);
      mean_error +=
          (transformed_source.AtUnsafe(index) - neighbor_cloud.AtUnsafe(index))
              .norm() /
          transformed_source.size();
    }

    // Update the accumulated transform by the transform belonging to the
    // current matched points.
    *dest_pose_source = dest_pose_transformed_source * *dest_pose_source;

    if (std::abs(prev_error - mean_error) < tolerance) {
      break;
    }
    prev_error = mean_error;
  }
  return mean_error;
}

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_ALGORITHMS_H_
