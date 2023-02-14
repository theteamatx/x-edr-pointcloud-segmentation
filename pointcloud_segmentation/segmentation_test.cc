#include "googlex/proxy/object_properties/point_cloud/segmentation.h"

#include <vector>

#include "googlex/proxy/object_properties/point_cloud/cloud.h"
#include "googlex/proxy/object_properties/point_cloud/planar_region.h"
#include "testing/base/public/gunit.h"

namespace blue::mobility {

static constexpr float kEpsilon = 1.0e-7f;

TEST(TestPointCloudSegmentation, FindSeedPoints) {
  constexpr int kSize = 5;
  CloudBuffer<eigenmath::Vector3f> points(kSize, kSize);
  CloudBuffer<eigenmath::Vector3f> normals(kSize, kSize);

  for (int col = 0; col < points.Cols(); ++col) {
    for (int row = 0; row < points.Rows(); ++row) {
      auto& point = points.AtUnsafe(row, col);
      auto& normal = normals.AtUnsafe(row, col);
      point.x() = 0.1f * col;
      point.y() = 0.1f * row;
      point.z() = 0.0f;
      normal.x() = 0.0f;
      normal.y() = 0.0f;
      normal.z() = 1.0f;
    }
  }

  std::vector<int> seed_point_indices =
      FindSeedPointsFromPlaneSupport(points, normals, 3, 0.05f, 6);
  EXPECT_EQ(seed_point_indices.size(), kSize * kSize - 4);

  seed_point_indices =
      FindSeedPointsFromPlaneSupport(points, normals, 5, 0.05f, 16);
  EXPECT_EQ(seed_point_indices.size(), (kSize - 2) * (kSize - 2));
}

}  // namespace blue::mobility
