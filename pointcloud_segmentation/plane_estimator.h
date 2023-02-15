
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

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_ESTIMATOR_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_ESTIMATOR_H_

#include <vector>

#include "googlex/proxy/eigenmath/pose3.h"
#include "googlex/proxy/object_properties/point_cloud/cloud.h"
#include "googlex/proxy/object_properties/point_cloud/indices.h"
#include "googlex/proxy/object_properties/point_cloud/proto/plane_estimator.proto.h"

namespace blue::mobility {

// PlaneEstimator efficiently keeps track of a running plane estimate from a
// set of points. Extending the set of points the plane estimate is only
// requires adjustment of the internal accumulators. The plane, centroid and
// curvature are lazily computed on the respective accessors, not in Merge or
// AddPoints. Note that the plane normal orientation stays the same between
// different plane estimations. It can be set with SetNormalOrientation and is
// kept from thereon.
class PlaneEstimator {
 public:
  PlaneEstimator();
  PlaneEstimator(const PlaneEstimatorProto& proto);

  // Clear all plane estimation accumulators.
  void Clear();

  // Get the plane centroid. If it's not up-to-date, it is recalculated.
  const eigenmath::Vector3f& Centroid() const;

  // Returns the curvature of the current plane. If it's not up-to-date, it is
  // recalculated.
  float Curvature() const;

  // Get the plane. If it's not up-to-date, it is recalculated.
  const eigenmath::Plane3f& Plane() const;

  // Indicates whether a valid plane has been computed. If the plane is not
  // up-to-date, it is recalculated.
  bool PlaneValid() const;

  // Merges another PlaneEstimator accumulation with this accumulation to
  // compute the plane based on the joint point set. Note that this does not
  // actually update the plane model. However, any call subsequent to
  // Curvature() Plane() or Centroid() will lazily recalculate the plane.
  void Merge(const PlaneEstimator& other);

  // Same as the Merge above, but allows for the other PlaneEstimator to be in a
  // different coordinate frame. The transform from the other coordinate frame
  // to the coordinate frame of this PlaneEstimator needs to be provided.
  void Merge(const PlaneEstimator& other,
             const eigenmath::Pose3d& this_pose_other);

  // Transforms the accumulator state from the current coordinate frame to
  // another coordinate frame.
  void Transform(const eigenmath::Pose3d& new_pose_current);

  // Adjusts the plane normal orientation towards the normal_orientation_hint.
  // Note that this just changes the plane normal sign and leaves the direction
  // unaltered. You can call this even before any points are added to the plane
  // estimator or at any later point. Subsequent plane estimations will have the
  // same normal orientation. If this is not set before Plane() is called, the
  // normal orientation is arbitrary.
  void SetNormalOrientation(const eigenmath::Vector3f& normal_orientation_hint);

  // Adds a point to the plane estimation. Note that this does not actually
  // update the plane model. However, any call subsequent to Curvature() Plane()
  // or Centroid() will lazily recalculate the plane.
  void AddPoint(const eigenmath::Vector3f& point);

  // Weighted version of the above AddPoint that allows to give points a
  // contribution that can for example be proportional to 1 / point_std_dev to
  // factor uncertainty in.
  void AddPoint(const eigenmath::Vector3f& point, float weight);

  // Vector version of AddPoint.
  void AddPoints(const std::vector<eigenmath::Vector3f>& points);

  // Cloud<T> convenience function for a point cloud for AddPoint.
  template <typename PointT, typename IndicesT = NoIndices>
  void AddPoints(const Cloud<PointT>& points,
                 const IndicesT& indices = NoIndices()) {
    for (int i = 0; i < GetIndicesSize(points, indices); ++i) {
      AddPoint(points.AtUnsafe(GetIndex(i, indices)));
    }
  }

  void ToProto(PlaneEstimatorProto* proto) const;

 private:
  void ComputePlane() const;

  // plane_, curvature_ and centroid_ need to be mutable, because we lazily
  // compute them when !plane_up_to_date_ in one of the accessors.
  mutable eigenmath::Plane3f plane_;
  mutable eigenmath::Vector3f centroid_;
  mutable float curvature_;
  mutable bool plane_up_to_date_;
  mutable bool plane_valid_;
  eigenmath::Vector3f cumulative_centroid_;
  eigenmath::Vector6f accumulator_;
  float cumulative_weights_;
};

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_ESTIMATOR_H_
