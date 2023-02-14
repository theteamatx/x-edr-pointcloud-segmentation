#include "googlex/proxy/object_properties/point_cloud/cloud.h"

#include <vector>

#include "googlex/proxy/eigenmath/matchers.h"
#include "testing/base/public/benchmark.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"

namespace blue::mobility {
namespace {

using ::blue::eigenmath::testing::IsApprox;
using ::testing::Eq;
using ::testing::NanSensitiveDoubleEq;
using ::testing::NanSensitiveFloatEq;

template <typename T>
T ReadOnlyAlgorithm(const Cloud<T>& cloud) {
  return cloud.AtUnsafe(0);
}

template <typename T>
void ReadWriteAlgorithm(Cloud<T>* cloud) {
  cloud->AtUnsafe(0) = cloud->AtUnsafe(1);
}

TEST(Cloud, AtBoundsChecks) {
  CloudBuffer<eigenmath::Vector3f> cloud(2, 2);
  ASSERT_DEATH(cloud.At(0, 3), "");
  ASSERT_DEATH(cloud.At(-1, 0), "");
}

TEST(Cloud, Sizes) { CloudBuffer<eigenmath::Vector3f> cloud(2, 2); }

TEST(Cloud, Resize) {
  CloudBuffer<eigenmath::Vector3f> cloud(0, 0);
  cloud.Resize(1, 2);
  cloud.AtUnsafe(0) = eigenmath::Vector3f(1, 2, 3);
  cloud.AtUnsafe(1) = eigenmath::Vector3f(4, 5, 6);
  cloud.Resize(2, 2);
  cloud.AtUnsafe(2) = eigenmath::Vector3f(7, 8, 9);
  cloud.AtUnsafe(3) = eigenmath::Vector3f(10, 11, 12);
  EXPECT_EQ(cloud.AtUnsafe(0), eigenmath::Vector3f(1, 2, 3));
  EXPECT_EQ(cloud.AtUnsafe(1), eigenmath::Vector3f(4, 5, 6));
  EXPECT_EQ(cloud.AtUnsafe(2), eigenmath::Vector3f(7, 8, 9));
  EXPECT_EQ(cloud.AtUnsafe(3), eigenmath::Vector3f(10, 11, 12));
  cloud.Resize(1, 2);
  EXPECT_EQ(cloud.AtUnsafe(0), eigenmath::Vector3f(1, 2, 3));
  EXPECT_EQ(cloud.AtUnsafe(1), eigenmath::Vector3f(4, 5, 6));
}

TEST(Cloud, Copy) {
  CloudBuffer<eigenmath::Vector3f> cloud(0, 0);
  {
    CloudBuffer<eigenmath::Vector3f> other(1, 2);
    other.Resize(1, 2);
    other.AtUnsafe(0) = eigenmath::Vector3f(1, 2, 3);
    other.AtUnsafe(1) = eigenmath::Vector3f(4, 5, 6);
    cloud = other;
  }
  EXPECT_EQ(cloud.AtUnsafe(0), eigenmath::Vector3f(1, 2, 3));
  EXPECT_EQ(cloud.AtUnsafe(1), eigenmath::Vector3f(4, 5, 6));
}

TEST(Cloud, RowMajorColMajor) {
  std::vector<int> data = {1, 2, 3, 4};

  CloudView<int> col_maj_view(data.data(), 2, 2);
  EXPECT_EQ(col_maj_view.At(0, 0), 1);
  EXPECT_EQ(col_maj_view.At(1, 0), 2);
  EXPECT_EQ(col_maj_view.At(0, 1), 3);
  EXPECT_EQ(col_maj_view.At(1, 1), 4);

  CloudView<int, cloud::RowMajor> row_maj_view(data.data(), 2, 2);
  EXPECT_EQ(row_maj_view.At(0, 0), 1);
  EXPECT_EQ(row_maj_view.At(1, 0), 3);
  EXPECT_EQ(row_maj_view.At(0, 1), 2);
  EXPECT_EQ(row_maj_view.At(1, 1), 4);

  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(col_maj_view.At(i), row_maj_view.At(i));
  }
}

TEST(Cloud, Index) {
  std::vector<int> data = {1, 2, 3, 4};
  CloudView<int> col_maj_view(data.data(), 2, 2);
  CloudView<int, cloud::RowMajor> row_maj_view(data.data(), 2, 2);
  for (int index = 0; index < 4; ++index) {
    int row, col;
    col_maj_view.IndexToRowCol(index, &row, &col);
    EXPECT_EQ(col_maj_view.LinearizeIndex(row, col), index);
    row_maj_view.IndexToRowCol(index, &row, &col);
    EXPECT_EQ(row_maj_view.LinearizeIndex(row, col), index);
  }
}

#define TEST_CLOUD_TYPE(name, type, v1, v2, matcher)          \
  TEST(Cloud, CommonTypes##name) {                            \
    CloudBuffer<type> cloud(2, 2);                            \
    cloud.AtUnsafe(0) = v1;                                   \
    cloud.AtUnsafe(1) = v2;                                   \
    EXPECT_THAT(ReadOnlyAlgorithm<type>(cloud), matcher(v1)); \
    CloudView<type> view(&cloud);                             \
    ReadWriteAlgorithm(&view);                                \
    EXPECT_THAT(ReadOnlyAlgorithm<type>(cloud), matcher(v2)); \
  }

TEST_CLOUD_TYPE(Int, int, 1, 2, Eq)
TEST_CLOUD_TYPE(Float, float, 5.0, std::numeric_limits<float>::quiet_NaN(),
                NanSensitiveFloatEq)
TEST_CLOUD_TYPE(Double, double, 5.0, std::numeric_limits<double>::infinity(),
                NanSensitiveDoubleEq)
TEST_CLOUD_TYPE(EigenmathVector3f, eigenmath::Vector3f,
                eigenmath::Vector3f(1, 2, 3), eigenmath::Vector3f(4, 5, 6),
                IsApprox)
TEST_CLOUD_TYPE(EigenmathVector3d, eigenmath::Vector3d,
                eigenmath::Vector3d(1, 2, 3), eigenmath::Vector3d(4, 5, 6),
                IsApprox)
TEST_CLOUD_TYPE(EigenVector2dFixedSizeVectorizable, Eigen::Vector2d,
                Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(3.0, 4.0), IsApprox)
TEST_CLOUD_TYPE(EigenVector4dFixedSizeVectorizable, Eigen::Vector4d,
                Eigen::Vector4d(1, 2, 3, 4), Eigen::Vector4d(5, 6, 7, 8),
                IsApprox)

#undef TEST_CLOUD_TYPE

}  // namespace
}  // namespace blue::mobility
