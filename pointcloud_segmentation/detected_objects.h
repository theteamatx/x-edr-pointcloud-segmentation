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

// Create a struct and proto to store the segmentation result.
#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_DETECTED_OBJECTS_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_DETECTED_OBJECTS_H_

#include <iterator>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "eigenmath/conversions.h"
#include "eigenmath/eigenmath.pb.h"
#include "eigenmath/pose3.h"
#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/cloud_proto_utils.h"
#include "pointcloud_segmentation/detected_objects.pb.h"
#include "pointcloud_segmentation/planar_region.h"

namespace mobility {

// ToProto and FromProto of Plane3f.
inline bool ToProto(const eigenmath::Plane3f& in_plane,
                    Plane3dProto* out_proto) {
  const eigenmath::Vector3f point_on_plane =
      -in_plane.normal() * in_plane.offset();
  out_proto->set_x(point_on_plane.x());
  out_proto->set_y(point_on_plane.y());
  out_proto->set_z(point_on_plane.z());
  out_proto->set_nx(in_plane.normal().x());
  out_proto->set_ny(in_plane.normal().y());
  out_proto->set_nz(in_plane.normal().z());

  return true;
}

inline bool FromProto(const Plane3dProto& in_proto,
                      eigenmath::Plane3f* out_data) {
  const eigenmath::Vector3f normal(in_proto.nx(), in_proto.ny(), in_proto.nz());
  const eigenmath::Vector3f point(in_proto.x(), in_proto.y(), in_proto.z());
  CHECK(normal.squaredNorm() > Eigen::NumTraits<float>::dummy_precision());

  *out_data = eigenmath::Plane3f(normal.normalized(), point);
  return true;
}

// Generate DetectedObjectProto from cluster_region and point_cloud
inline void CreateClusterDetectedObjectProto(
    const Cloud<eigenmath::Vector3f>& point_cloud,
    const Indices& inlier_indices, absl::string_view object_class,
    DetectedObjectProto* out_proto) {
  out_proto->set_object_class(object_class);
  auto* cluster_geometry = out_proto->mutable_cluster_geometry();
  auto points_as_cloud = GetOrCreateCloudView<eigenmath::Vector3f>(
      1, inlier_indices.size(), 0.0f, cluster_geometry->mutable_points_xyz());
  auto points_out_it = points_as_cloud.begin();
  for (int index : inlier_indices) {
    *points_out_it++ = point_cloud.AtUnsafe(index);
  }
}

// Generate DetectedObjectProto from planar_region, plane class and point_cloud
void CreatePlanarDetectedObjectProto(
    const PlanarRegion<>& planar_region,
    const Cloud<eigenmath::Vector3f>& point_cloud,
    absl::string_view object_class, DetectedObjectProto* out_proto);

}  // namespace mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_DETECTED_OBJECTS_H_
