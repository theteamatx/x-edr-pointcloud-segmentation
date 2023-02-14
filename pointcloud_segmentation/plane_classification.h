// Provides several simple functions to classify planes geometrically. The
// current supported classes are Floor, Wall and Coffeetable.

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_CLASSIFICATION_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_PLANE_CLASSIFICATION_H_

#include <string>

#include "googlex/proxy/eigenmath/types.h"
#include "googlex/proxy/object_properties/point_cloud/planar_region.h"
#include "googlex/proxy/object_properties/point_cloud/plane_classification_config.proto.h"

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
