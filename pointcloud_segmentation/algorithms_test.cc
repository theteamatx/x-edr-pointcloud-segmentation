#include "googlex/proxy/object_properties/point_cloud/algorithms.h"

#include <vector>

#include "devtools/build/runtime/get_runfiles_dir.h"
#include "file/base/helpers.h"
#include "googlex/proxy/eigenmath/matchers.h"
#include "googlex/proxy/eigenmath/pose3.h"
#include "googlex/proxy/eigenmath/pose3_utils.h"
#include "googlex/proxy/eigenmath/so3.h"
#include "googlex/proxy/eigenmath/types.h"
#include "googlex/proxy/object_properties/point_cloud/cloud.h"
#include "googlex/proxy/object_properties/point_cloud/multichannel_cloud.h"
#include "googlex/proxy/object_properties/point_cloud/multichannel_cloud.proto.h"
#include "testing/base/public/benchmark.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"
#include "util/math/mathutil.h"

namespace blue::mobility {
namespace {

using ::blue::eigenmath::testing::IsApprox;
using ::testing::Message;

constexpr float kEpsilon = 1.0e-4f;

TEST(TestPointCloudAlgorithms, IsInsideTriangleClockwise) {
  Eigen::Vector3f u(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
  Eigen::Vector3f w(1.0f, -1.0f, 0.0f);
  Eigen::Vector3f p(0.2f, -0.1f, 0.0f);

  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.z() = 1.0;
  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.z() = -1.0;
  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.y() = 0.1;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.x() = 1.1;
  p.y() = -0.1;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.x() = 0.1;
  p.y() = -0.2;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
}

TEST(TestPointCloudAlgorithms, IsInsideTriangleCounterClockwise) {
  Eigen::Vector3f u(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
  Eigen::Vector3f w(1.0f, 1.0f, 0.0f);
  Eigen::Vector3f p(0.2f, 0.1f, 0.0f);

  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.z() = 1.0;
  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.z() = -1.0;
  EXPECT_TRUE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.y() = -0.1;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.x() = 1.1;
  p.y() = 0.1;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
  p.x() = 0.1;
  p.y() = 0.2;
  EXPECT_FALSE(blue::mobility::detail::IsInsideTriangle(u, v, w, p));
}

TEST(TestPointCloudAlgorithms, IsEar) {
  // The test polygon looks something like this zig zag:
  //        1
  // 3      /\
  //  \`. 2/  \
  //   \ `' /  \
  //    \  /5`. \
  //     \/    `.\
  //     0        4
  CloudBuffer<eigenmath::Vector3f> points(6);
  points.AtUnsafe(0) = {0.0f, 0.0f, 0.0f};
  points.AtUnsafe(1) = {1.0f, 1.0f, 0.0f};
  points.AtUnsafe(2) = {0.0f, 0.1f, 0.0f};
  points.AtUnsafe(3) = {-1.0f, 1.0f, 0.0f};
  points.AtUnsafe(4) = {2.0f, 0.0f, 0.0f};
  points.AtUnsafe(5) = {1.0f, 0.8f, 0.0f};
  std::vector<int> polygon{0, 3, 2, 1, 4, 5};
  Eigen::Vector3f normal =
      blue::mobility::CumulativePolygonNormal(points, polygon);

  // Initial ear clipping candidates
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 0, 1, 2, polygon, normal));
  EXPECT_FALSE(blue::mobility::detail::IsEar(points, 1, 2, 3, polygon, normal));
  EXPECT_FALSE(blue::mobility::detail::IsEar(points, 2, 3, 4, polygon, normal));
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 3, 4, 5, polygon, normal));
  EXPECT_FALSE(blue::mobility::detail::IsEar(points, 4, 5, 0, polygon, normal));
  EXPECT_FALSE(blue::mobility::detail::IsEar(points, 5, 0, 1, polygon, normal));

  // Other possible combinations
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 0, 2, 3, polygon, normal));
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 2, 3, 0, polygon, normal));
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 2, 3, 5, polygon, normal));
  EXPECT_TRUE(blue::mobility::detail::IsEar(points, 5, 0, 2, polygon, normal));
}

TEST(TestPointCloudAlgorithms, PolygonNormalAndArea) {
  CloudBuffer<eigenmath::Vector3f> points(3);
  points.AtUnsafe(0) = {0.0f, 0.0f, 1000.0f};
  points.AtUnsafe(1) = {1.0f, 1.0f, 1000.0f};
  points.AtUnsafe(2) = {1.0f, 0.0f, 1000.0f};

  Eigen::Vector3f normal = blue::mobility::PolygonNormal(points);
  EXPECT_NEAR(0.0f, normal.x(), kEpsilon);
  EXPECT_NEAR(0.0f, normal.y(), kEpsilon);
  EXPECT_NEAR(-1.0f, normal.z(), kEpsilon);
  EXPECT_NEAR(0.5f, blue::mobility::PolygonArea(points), kEpsilon);
  // Test correct zero area behavior by adding indices that don't add area
  EXPECT_NEAR(
      0.5f,
      blue::mobility::PolygonArea(points, std::vector<int>{0, 1, 2, 1, 2}),
      kEpsilon);

  points.AtUnsafe(1).y() = 0.0f;
  points.AtUnsafe(1).z() = 999.0f;
  normal = blue::mobility::PolygonNormal(points);
  EXPECT_NEAR(0.0f, normal.x(), kEpsilon);
  EXPECT_NEAR(-1.0f, normal.y(), kEpsilon);
  EXPECT_NEAR(0.0f, normal.z(), kEpsilon);
  EXPECT_NEAR(0.5f, blue::mobility::PolygonArea(points), kEpsilon);
}

TEST(TestPointCloudAlgorithms, TriangulateLoopyPolygon) {
  // The test polygon looks something like this zig zag:
  //        1
  // 3      /\
  //  \`. 2/  \
  //   \ `' /  \
  //    \  /5`. \
  //     \/    `.\
  //     0        4
  CloudBuffer<eigenmath::Vector3f> points(6);
  points.AtUnsafe(0) = {0.0f, 0.0f, 0.0f};
  points.AtUnsafe(1) = {1.0f, 1.0f, 0.0f};
  points.AtUnsafe(2) = {0.0f, 0.1f, 0.0f};
  points.AtUnsafe(3) = {-1.0f, 1.0f, 0.0f};
  points.AtUnsafe(4) = {2.0f, 0.0f, 0.0f};
  points.AtUnsafe(5) = {1.0f, 0.8f, 0.0f};

  std::vector<int> polygon{0, 3, 2, 1, 4, 5};
  std::vector<int> triangles;
  EXPECT_TRUE(
      blue::mobility::TriangulateLoopyPolygon(points, polygon, &triangles));
  EXPECT_EQ(triangles.size(), (points.size() - 2) * 3);

  float area = 0.0f;
  for (int i = 0; i < triangles.size(); i += 3) {
    area += blue::mobility::PolygonArea(
        points,
        std::vector<int>{triangles[i], triangles[i + 1], triangles[i + 2]});
  }
  EXPECT_NEAR(area, blue::mobility::PolygonArea(points, polygon), kEpsilon);

  // add a zero area polygon part with duplicated indices
  polygon = {0, 3, 2, 1, 4, 5, 4, 1, 4, 5};
  EXPECT_TRUE(
      blue::mobility::TriangulateLoopyPolygon(points, polygon, &triangles));
  EXPECT_EQ(triangles.size(), (points.size() - 2) * 3);

  area = 0.0f;
  for (int i = 0; i < triangles.size(); i += 3) {
    area += blue::mobility::PolygonArea(
        points,
        std::vector<int>{triangles[i], triangles[i + 1], triangles[i + 2]});
  }
  EXPECT_NEAR(area, blue::mobility::PolygonArea(points, polygon), kEpsilon);
}

TEST(TestPointCloudAlgorithms, TriangulatePolygonRealCase) {
  CloudBuffer<eigenmath::Vector3f> points(14);
  points.AtUnsafe(0) = {1.77995, 5.67707, 0.00866};
  points.AtUnsafe(1) = {1.71035, 5.36595, 0.00898};
  points.AtUnsafe(2) = {1.64175, 5.06695, 0.00933};
  points.AtUnsafe(3) = {2.11661, 5.73814, -0.00001};
  points.AtUnsafe(4) = {2.02174, 5.40343, 0.00087};
  points.AtUnsafe(5) = {1.94777, 5.13560, 0.00151};
  points.AtUnsafe(6) = {1.86870, 4.85730, 0.00225};
  points.AtUnsafe(7) = {1.45473, 4.22188, 0.01014};
  points.AtUnsafe(8) = {1.49240, 4.39640, 0.01000};
  points.AtUnsafe(9) = {1.54583, 4.62679, 0.00971};
  points.AtUnsafe(10) = {1.59422, 4.84672, 0.00951};
  points.AtUnsafe(11) = {1.64174, 5.06695, 0.00933};
  points.AtUnsafe(12) = {1.71034, 5.36595, 0.00898};
  points.AtUnsafe(13) = {1.33166, 4.81384, 0.01635};

  std::vector<int> polygon{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
  std::vector<int> triangles;
  EXPECT_TRUE(blue::mobility::TriangulatePolygon(points, polygon, &triangles));
  EXPECT_EQ(triangles.size(), (polygon.size() - 2) * 3);

  float area = 0.0f;
  for (int i = 0; i < triangles.size(); i += 3) {
    area += blue::mobility::PolygonArea(
        points,
        std::vector<int>{triangles[i], triangles[i + 1], triangles[i + 2]});
  }
  EXPECT_NEAR(area, blue::mobility::PolygonArea(points, polygon), kEpsilon);
}

CloudBuffer<eigenmath::Vector3f> BuildDensePointCloud(int size_x, int size_y,
                                                      double x, double y) {
  CloudBuffer<eigenmath::Vector3f> points(size_x, size_y);

  for (int i = 0; i < size_x; ++i) {
    for (int j = 0; j < size_y; ++j) {
      points.AtUnsafe(j, i) = {static_cast<float>(i * x / size_x),
                               static_cast<float>(j * y / size_y), 1.0};
    }
  }
  return points;
}

TEST(TestPointCloudAlgorithms, FindNormalSupportNeighbors) {
  CloudBuffer<eigenmath::Vector3f> points =
      BuildDensePointCloud(100, 100, 10.0, 10.0);

  std::vector<int> indices;
  indices.resize(9);
  EXPECT_EQ(9, blue::mobility::detail::FindNormalSupportNeighbors(
                   points, 20, 20, 0.05, 0.4, true, &indices));
  EXPECT_THAT(indices, testing::Contains(21 * 100 + 20));
  EXPECT_THAT(indices, testing::Contains(20 * 100 + 21));
  EXPECT_THAT(indices, testing::Contains(19 * 100 + 20));
  EXPECT_THAT(indices, testing::Contains(20 * 100 + 19));
  EXPECT_THAT(indices, testing::Contains(20 * 100 + 20));
}

// See go/benchmarks for information about defining benchmarks.
void BM_FindNormalSupportNeighbors(benchmark::State& state) {
  CloudBuffer<eigenmath::Vector3f> points =
      BuildDensePointCloud(100, 100, 10.0, 10.0);

  const double step = 10.0 / 100;
  const double min = state.range(0) * step;
  const double max = state.range(1) * step;

  std::vector<int> indices;
  indices.resize(9);
  for (auto _ : state) {
    blue::mobility::detail::FindNormalSupportNeighbors(points, 50, 50, min, max,
                                                       true, &indices);
  }
}
BENCHMARK(BM_FindNormalSupportNeighbors)
    ->ArgPair(0, 4)
    ->ArgPair(1, 4)
    ->ArgPair(2, 4)
    ->ArgPair(3, 4)
    ->ArgPair(1, 100)
    ->ArgPair(50, 100)
    ->ArgPair(90, 100);

TEST(TestPointCloudAlgorithms, NearestNeighbors) {
  const int num_points = 10;
  CloudBuffer<eigenmath::Vector3f> source(num_points);
  CloudBuffer<eigenmath::Vector3f> dest(num_points);

  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3f point(i, i, i);
    source.AtUnsafe(i) = point;
    dest.AtUnsafe(num_points - i - 1) = point;
  }

  std::vector<int> nearest_neighbors;
  NearestNeighbors(source, dest, &nearest_neighbors);
  ASSERT_EQ(source.size(), nearest_neighbors.size());
  for (int i = 0; i < nearest_neighbors.size(); i++) {
    ASSERT_EQ(nearest_neighbors[i], num_points - i - 1);
  }
}

TEST(TestPointCloudAlgorithms, ComputeRigidTransformTranslation) {
  const int num_points = 10;
  const eigenmath::Pose3f dest_pose_source_expected =
      eigenmath::Pose3f(eigenmath::ExpSO3(eigenmath::Vector3f(0, 0, 0)),
                        eigenmath::Vector3f(1.0, 2.0, 3.0));

  CloudBuffer<eigenmath::Vector3f> source(num_points);
  CloudBuffer<eigenmath::Vector3f> dest(num_points);

  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3f point(i, i, i);
    source.AtUnsafe(i) = point;
    dest.AtUnsafe(i) = dest_pose_source_expected * point;
  }

  eigenmath::Pose3f dest_pose_source_actual;
  ASSERT_TRUE(ComputeRigidTransform(source, dest, &dest_pose_source_actual));
  ASSERT_THAT(dest_pose_source_actual,
              eigenmath::testing::IsApprox(dest_pose_source_expected));
}

TEST(TestPointCloudAlgorithms, ComputeRigidTransformRotation) {
  const int num_points = 10;
  const eigenmath::Pose3f dest_pose_source_expected =
      eigenmath::Pose3f(eigenmath::ExpSO3(eigenmath::Vector3f(0, 0, M_PI_4)),
                        eigenmath::Vector3f(0.0, 0.0, 0.0));

  CloudBuffer<eigenmath::Vector3f> source(num_points);
  CloudBuffer<eigenmath::Vector3f> dest(num_points);

  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3f point(i, i, i);
    source.AtUnsafe(i) = point;
    dest.AtUnsafe(i) = dest_pose_source_expected * point;
  }

  eigenmath::Pose3f dest_pose_source_actual;
  ASSERT_TRUE(ComputeRigidTransform(source, dest, &dest_pose_source_actual));
  ASSERT_THAT(dest_pose_source_actual,
              eigenmath::testing::IsApprox(dest_pose_source_expected, 1e-3));
}

TEST(TestPointCloudAlgorithms, ComputeRigidTransformTranslationRotation) {
  const int num_points = 10;
  const eigenmath::Pose3f dest_pose_source_expected =
      eigenmath::Pose3f(eigenmath::ExpSO3(eigenmath::Vector3f(0, 0, M_PI_4)),
                        eigenmath::Vector3f(1.0, 2.0, 3.0));

  CloudBuffer<eigenmath::Vector3f> source(num_points);
  CloudBuffer<eigenmath::Vector3f> dest(num_points);

  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3f point(i, i, i);
    source.AtUnsafe(i) = point;
    dest.AtUnsafe(i) = dest_pose_source_expected * point;
  }

  eigenmath::Pose3f dest_pose_source_actual;
  ASSERT_TRUE(ComputeRigidTransform(source, dest, &dest_pose_source_actual));
  ASSERT_THAT(dest_pose_source_actual,
              eigenmath::testing::IsApprox(dest_pose_source_expected));
}

TEST(TestPointCloudAlgorithms, IcpOnSamePointCloud) {
  CloudBuffer<eigenmath::Vector3f> points =
      BuildDensePointCloud(10, 10, 10.0, 10.0);
  eigenmath::Pose3f dest_pose_source_actual;
  const eigenmath::Pose3f identity;
  ASSERT_NEAR(IterativeClosestPoint(points, points, &dest_pose_source_actual),
              0, 1e-3);
  ASSERT_THAT(dest_pose_source_actual, eigenmath::testing::IsApprox(identity));
}

TEST(TestPointCloudAlgorithms, IcpOnShuffledPointCloud) {
  const int num_points = 10;
  CloudBuffer<eigenmath::Vector3f> source(num_points);
  CloudBuffer<eigenmath::Vector3f> dest(num_points);

  for (int i = 0; i < num_points; i++) {
    const eigenmath::Vector3f point(i, i, i);
    source.AtUnsafe(i) = point;
    dest.AtUnsafe(num_points - i - 1) = point;
  }

  eigenmath::Pose3f dest_pose_source_actual;
  const eigenmath::Pose3f identity;
  ASSERT_NEAR(IterativeClosestPoint(source, dest, &dest_pose_source_actual), 0,
              1e-3);
  ASSERT_THAT(dest_pose_source_actual,
              eigenmath::testing::IsApprox(identity, 1e-3));
}

TEST(TestPointCloudAlgorithms, IcpMultipleIterations) {
  const int num_points = 5;

  CloudBuffer<eigenmath::Vector3f> source(num_points);
  source.AtUnsafe(0) = eigenmath::Vector3f(1, 1, 0);
  source.AtUnsafe(1) = eigenmath::Vector3f(2, 1, 0);
  source.AtUnsafe(2) = eigenmath::Vector3f(2, 2, 0);
  source.AtUnsafe(3) = eigenmath::Vector3f(5, 2, 1);
  source.AtUnsafe(4) = eigenmath::Vector3f(-4, 0, -3);

  const eigenmath::Pose3f dest_pose_source_expected =
      eigenmath::Translation(3.0, 0.0, 0.0).cast<float>();

  CloudBuffer<eigenmath::Vector3f> dest(num_points);
  for (int i = 0; i < num_points; i++) {
    dest.AtUnsafe(i) = dest_pose_source_expected * source.AtUnsafe(i);
  }

  eigenmath::Pose3f dest_pose_source_actual;
  ASSERT_NEAR(IterativeClosestPoint(source, dest, &dest_pose_source_actual), 0,
              1e-3);
  ASSERT_THAT(dest_pose_source_actual,
              eigenmath::testing::IsApprox(dest_pose_source_expected, 1e-3));
}

template <typename Scalar>
class AlgorithmsTest : public testing::Test {};

using Scalars = ::testing::Types<float, double>;
TYPED_TEST_SUITE(AlgorithmsTest, Scalars);

TYPED_TEST(AlgorithmsTest, ComputeRigidTransform) {
  CloudBuffer<eigenmath::Vector3<TypeParam>> cloud_a(4);
  cloud_a.At(0) = eigenmath::Vector3<TypeParam>(3, 2, 9);
  cloud_a.At(1) = eigenmath::Vector3<TypeParam>(-2, 42, 2);
  cloud_a.At(2) = eigenmath::Vector3<TypeParam>(13, 3, -1);
  cloud_a.At(3) = eigenmath::Vector3<TypeParam>(-8, -2, -7);
  const eigenmath::Pose3<TypeParam> b_pose_a(
      eigenmath::SO3<TypeParam>(0.3, -0.5, 4.2),
      eigenmath::Vector3<TypeParam>(3, 4, 5));
  CloudBuffer<eigenmath::Vector3<TypeParam>> cloud_b(cloud_a.Size());
  for (int i = 0; i < cloud_a.Size(); ++i) {
    cloud_b.At(i) = b_pose_a * cloud_a.At(i);
  }
  eigenmath::Pose3<TypeParam> b_pose_a_computed;
  ASSERT_TRUE(ComputeRigidTransform(cloud_a, cloud_b, &b_pose_a_computed));
  EXPECT_THAT(b_pose_a_computed, IsApprox(b_pose_a, 1e-5));
}

TEST(ComputeNormalsOrganizedTest, AllValidNormalsAreNormalized) {
  ASSERT_OK_AND_ASSIGN(auto cloud_proto,
                       file::GetBinaryProto<MultichannelCloudProto>(
                           devtools_build::GetDataDependencyFilepath(
                               "google3/googlex/proxy/object_properties/"
                               "point_cloud/testdata/"
                               "cbr_multichannel_cloud_proto.pb"),
                           file::Defaults()));

  MultichannelCloudView cloud_view(&cloud_proto);
  const auto points = cloud_view.GetPoints();
  CloudView<eigenmath::Vector3f> normals = cloud_view.GetOrCreateNormals();

  // Compute the normals for each point.
  ComputeNormalsOrganized(cloud_view.PointCloudPoseSensor(), points, &normals);

  // Verify that for each finite point and normal, the normal vector is
  // normalized.
  for (int col = 0; col < cloud_view.Cols(); ++col) {
    for (int row = 0; row < cloud_view.Rows(); ++row) {
      const eigenmath::Vector3f normal = normals.AtUnsafe(row, col);
      if (!points.AtUnsafe(row, col).allFinite() || !normal.allFinite())
        continue;

      SCOPED_TRACE(Message() << "Col: " << col << " Row: " << row);
      EXPECT_TRUE(MathUtil::AlmostEquals(normal.squaredNorm(), 1.0f))
          << normal.squaredNorm();
    }
  }
}

}  // namespace
}  // namespace blue::mobility
