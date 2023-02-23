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

#include "pointcloud_segmentation/detected_objects.h"

namespace mobility {

void CreatePlanarDetectedObjectProto(
    const PlanarRegion<>& planar_region,
    const Cloud<eigenmath::Vector3f>& point_cloud,
    const absl::string_view object_class, DetectedObjectProto* out_proto) {
  out_proto->set_object_class(object_class);
  auto* planar_geometry = out_proto->mutable_planar_geometry();
  const auto& plane_estimator = planar_region.plane_estimator();
  const auto& inlier_indices = planar_region.inlier_indices();
  const auto& discontinuous_boundary_indices =
      planar_region.discontinuous_boundary_indices();
  *planar_geometry->mutable_centroid() =
      eigenmath::conversions::ProtoFromVector3d(
          plane_estimator.Centroid().cast<double>());
  ToProto(plane_estimator.Plane(), planar_geometry->mutable_plane());

  auto points_as_cloud = GetOrCreateCloudView<eigenmath::Vector3f>(
      1, inlier_indices.size(), 0.0f, planar_geometry->mutable_points_xyz());

  auto points_out_it = points_as_cloud.begin();
  for (int index : inlier_indices) {
    *points_out_it++ = point_cloud.AtUnsafe(index);
    if (discontinuous_boundary_indices.find(index) !=
        discontinuous_boundary_indices.end()) {
      planar_geometry->add_discontinuous_boundary_indices(
          std::distance(points_as_cloud.begin(), points_out_it) - 1);
    }
  }
}

}  // namespace mobility
