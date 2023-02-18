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

// Provides several simple functions to classify planes geometrically. The
// current supported classes are Floor, Wall and Coffeetable.

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_CLASSIFICATION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_CLASSIFICATION_H_

#include <string>

#include "eigenmath/types.h"
#include "pointcloud_segmentation/planar_region.h"
#include "pointcloud_segmentation/plane_classification_config.pb.h"

namespace blue::mobility {

struct PlaneClassificationDebugSummary {
  int total_considered = 0;
  struct HorizontalPlaneRejections {
    int rejected_for_angle = 0;
    int rejected_for_distance = 0;
    int rejected_for_size = 0;
    // A full multiline description of the plane rejection statistics.
    std::string PlaneRejectionReport() const;
  };
  HorizontalPlaneRejections floor_rejections;
  HorizontalPlaneRejections coffee_table_rejections;

  // A full multiline description of the classification statistics.
  std::string FullReport() const;
};

// Returns the plane class by doing simple geometric classification.
void ClassifyPlane(const PlaneClassificationConfigProto& config,
                   const eigenmath::Vector3f& robot_normal_direction,
                   const eigenmath::Vector3f& known_floor_point,
                   PlanarRegion<>* planar_region,
                   PlaneClassificationDebugSummary* summary);

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_CLASSIFICATION_H_
