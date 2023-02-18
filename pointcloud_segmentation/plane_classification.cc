#include "pointcloud_segmentation/plane_classification.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace blue::mobility {

namespace {

double DegToRad(const double angle_degrees) {
  return angle_degrees * (M_PI / 180.0);
}

// Checks the plane normal against the floor_normal_direction with
// cos_max_up_direction_delta_angle as threshold and distance offset.
bool IsHorizontalPlaneWithOffset(
    const ClassifyHorizontalPlaneParams& params,
    const PlanarRegion<>& planar_region,
    const eigenmath::Vector3f& up_direction,
    const eigenmath::Vector3f& known_floor_point,
    PlaneClassificationDebugSummary::HorizontalPlaneRejections* summary) {
  const float cos_max_up_direction_delta_angle =
      std::cos(DegToRad(params.max_up_direction_delta_angle_degrees()));

  if (planar_region.plane().normal().dot(up_direction) <
      cos_max_up_direction_delta_angle) {
    ++summary->rejected_for_angle;
    return false;
  }

  float floor_offset = planar_region.plane().signedDistance(known_floor_point);

  if (std::abs(params.floor_offset() + floor_offset) >
      params.max_floor_offset_deviation()) {
    ++summary->rejected_for_distance;
    return false;
  }

  // Check whether planar region area is larger than configured area.
  if (planar_region.area() < params.min_area() ||
      planar_region.area() > params.max_area()) {
    ++summary->rejected_for_size;
    return false;
  }

  return true;
}

// Returns true if the plane is sufficiently vertical and at least 1.5m tall.
bool IsWall(const ClassifyWallParams& params,
            const PlanarRegion<>& planar_region,
            const eigenmath::Vector3f& up_direction) {
  const float cos_max_horizontal_delta_angle =
      std::cos(DegToRad(90.0 - params.max_horizontal_delta_angle_degrees()));
  if (std::abs(planar_region.plane().normal().dot(up_direction)) >
      cos_max_horizontal_delta_angle) {
    return false;
  }

  float min_height = std::numeric_limits<float>::max();
  float max_height = -std::numeric_limits<float>::max();
  for (const eigenmath::Vector3f& point :
       planar_region.projected_boundary_points()) {
    float height = up_direction.dot(point);
    min_height = std::min(min_height, height);
    max_height = std::max(max_height, height);
  }

  // Walls must be at least a minimum height.
  if ((max_height - min_height) < params.min_height()) {
    return false;
  }

  return true;
}

}  // namespace

std::string PlaneClassificationDebugSummary::HorizontalPlaneRejections::
    PlaneRejectionReport() const {
  return absl::StrCat("regions rejection:", rejected_for_angle, " for angle, ",
                      rejected_for_distance, " for distance, ",
                      rejected_for_size, " for size.");
}

std::string PlaneClassificationDebugSummary::FullReport() const {
  return absl::StrCat("Considered ", total_considered, " planes:\n Floor ",
                      floor_rejections.PlaneRejectionReport(),
                      "\n Coffee Table ",
                      coffee_table_rejections.PlaneRejectionReport());
}

void ClassifyPlane(const PlaneClassificationConfigProto& config,
                   const eigenmath::Vector3f& robot_normal_direction,
                   const eigenmath::Vector3f& known_floor_point,
                   PlanarRegion<>* planar_region,
                   PlaneClassificationDebugSummary* summary) {
  ++summary->total_considered;
  if (IsHorizontalPlaneWithOffset(config.floor_params(), *planar_region,
                                  robot_normal_direction, known_floor_point,
                                  &summary->floor_rejections)) {
    // Is the plane a floor?
    *planar_region->mutable_plane_class() = PlaneClass::kFloor;
  } else if (IsHorizontalPlaneWithOffset(config.coffee_table_params(),
                                         *planar_region, robot_normal_direction,
                                         known_floor_point,
                                         &summary->coffee_table_rejections)) {
    // Is the plane a coffee table?
    *planar_region->mutable_plane_class() =
        PlaneClass::kTable;
  } else if (IsWall(config.wall_params(), *planar_region,
                    robot_normal_direction)) {
    // Is the plane a wall?
    *planar_region->mutable_plane_class() = PlaneClass::kWall;
  } else {
    // Default
    *planar_region->mutable_plane_class() = PlaneClass::kUnknown;
  }
}

}  // namespace blue::mobility
