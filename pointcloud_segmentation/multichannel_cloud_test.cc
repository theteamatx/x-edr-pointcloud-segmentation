#include "pointcloud_segmentation/multichannel_cloud.h"

#include <limits>

#include "benchmark/benchmark.h"
#include "eigenmath/eigenmath.pb.h"
#include "eigenmath/matchers.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace blue::mobility {

namespace {

using eigenmath::testing::IsApprox;

MultichannelCloudBuffer MakeEmptyCloud() {
  return MultichannelCloudBuffer(2, 2);
}

TEST(MultichannelCloudTest, Copy) {
  MultichannelCloudBuffer buffer(0, 0);
  {
    MultichannelCloudBuffer other(1, 2);
    auto points = other.GetOrCreatePoints();
    points.AtUnsafe(0) = eigenmath::Vector3f(1, 2, 3);
    points.AtUnsafe(1) = eigenmath::Vector3f(2, 3, 4);
    buffer = other;
  }
  auto points = buffer.GetPoints();
  EXPECT_EQ(points.AtUnsafe(0), eigenmath::Vector3f(1, 2, 3));
  EXPECT_EQ(points.AtUnsafe(1), eigenmath::Vector3f(2, 3, 4));
}

TEST(MultichannelCloudTest, HasProperties) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();
  EXPECT_FALSE(cloud.HasPoints());
  EXPECT_FALSE(cloud.HasNormals());
  EXPECT_FALSE(cloud.HasIntensities());
  EXPECT_FALSE(cloud.HasColors());
  EXPECT_FALSE(cloud.HasCbrOrigins());
  EXPECT_FALSE(cloud.HasReturnPulseWidths());
  EXPECT_FALSE(cloud.HasReturnIntensities());
  EXPECT_FALSE(cloud.HasReturnRanges());
}

TEST(MultichannelCloudTest, MissingPropertiesEmptyClouds) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();
  EXPECT_FALSE(cloud.HasPoints());
  CloudView<eigenmath::Vector3f> points = cloud.GetPoints();
  EXPECT_EQ(points.Rows(), 0);
  EXPECT_EQ(points.Cols(), 0);
  EXPECT_EQ(points.Size(), 0);
}

TEST(MultichannelCloudTest, GetOrCreate) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();
  CloudView<eigenmath::Vector3f> points = cloud.GetOrCreatePoints(5.0f);

  EXPECT_EQ(points.Rows(), 2);
  EXPECT_EQ(points.Cols(), 2);
  EXPECT_EQ(points.Size(), 4);

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_THAT(points.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(5.f, 5.f, 5.f)));
    }
  }
}

TEST(MultichannelCloudTest, AttributesGetCreateResize) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();

  constexpr int kRowMax = 16, kColMax = 21;

  // Test GetOrCreate() and Resize() operations work on the return_range
  // attribute.
  cloud.Resize(kRowMax - 1, kColMax - 2);
  CloudView<eigenmath::Vector3f> return_ranges =
      cloud.GetOrCreateReturnRanges();
  EXPECT_EQ(return_ranges.Rows(), kRowMax - 1);
  EXPECT_EQ(return_ranges.Cols(), kColMax - 2);
  EXPECT_EQ(return_ranges.Size(), (kRowMax - 1) * (kColMax - 2));

  cloud.Resize(kRowMax, kColMax);
  return_ranges = cloud.GetOrCreateReturnRanges();
  EXPECT_EQ(return_ranges.Rows(), kRowMax);
  EXPECT_EQ(return_ranges.Cols(), kColMax);
  EXPECT_EQ(return_ranges.Size(), kRowMax * kColMax);
}

TEST(MultichannelCloudTest, MultipleViews) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();
  CloudView<eigenmath::Vector3f> points_1 = cloud.GetOrCreatePoints(1.f);
  CloudView<eigenmath::Vector3f> points_2 = cloud.GetOrCreatePoints(2.f);
  CloudView<eigenmath::Vector3f> points_view = cloud.GetPoints();

  const MultichannelCloudBuffer& const_cloud = cloud;
  CloudView<const eigenmath::Vector3f> const_view = const_cloud.GetPoints();

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_THAT(points_1.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(1.f, 1.f, 1.f)));
      EXPECT_THAT(points_2.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(1.f, 1.f, 1.f)));
      EXPECT_THAT(points_view.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(1.f, 1.f, 1.f)));
      EXPECT_THAT(const_view.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(1.f, 1.f, 1.f)));
    }
  }

  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      points_1.AtUnsafe(row, col) = eigenmath::Vector3f(5.f, 4.f, 3.f);
      EXPECT_THAT(points_1.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(5.f, 4.f, 3.f)));
      EXPECT_THAT(points_2.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(5.f, 4.f, 3.f)));
      EXPECT_THAT(points_view.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(5.f, 4.f, 3.f)));
      EXPECT_THAT(const_view.AtUnsafe(row, col),
                  IsApprox(eigenmath::Vector3f(5.f, 4.f, 3.f)));
    }
  }
}

TEST(MultichannelCloudTest, ClearProperties) {
  MultichannelCloudBuffer cloud = MakeEmptyCloud();
  EXPECT_FALSE(cloud.HasPoints());
  cloud.GetOrCreatePoints();
  EXPECT_TRUE(cloud.HasPoints());
  cloud.ClearPoints();
  EXPECT_FALSE(cloud.HasPoints());
  cloud.GetOrCreateReturnRanges();
  EXPECT_TRUE(cloud.HasReturnRanges());
  cloud.ClearReturnRanges();
  EXPECT_FALSE(cloud.HasReturnRanges());
}

TEST(MultichannelCloudTest, ProtoMemoryMapping) {
  MultichannelCloudProto proto;
  proto.set_width(2);
  proto.set_height(2);
  MultichannelCloudView view(&proto);

  {
    MultichannelCloudProto expected;
    google::protobuf::TextFormat::ParseFromString(R"""(
      width: 2; height: 2
    )""",
                                                  &expected);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(
        proto, expected));
  }

  view.GetOrCreatePoints();
  {
    MultichannelCloudProto expected;
    google::protobuf::TextFormat::ParseFromString(R"""(
      width: 2; height: 2; points_xyz: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    )""",
                                                  &expected);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(
        proto, expected));
  }

  view.ClearPoints();
  {
    EXPECT_FALSE(view.HasPoints());
    MultichannelCloudProto expected;
    google::protobuf::TextFormat::ParseFromString(R"""(
      width: 2; height: 2
    )""",
                                                  &expected);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(
        proto, expected));
  }

  auto points = view.GetOrCreatePoints();
  for (int col = 0; col < 2; ++col) {
    for (int row = 0; row < 2; ++row) {
      constexpr float nan = std::numeric_limits<float>::quiet_NaN();
      points.AtUnsafe(row, col) = eigenmath::Vector3f(row, col, nan);
    }
  }

  for (int i = 0; i < proto.points_xyz_size(); ++i) {
    proto.set_points_xyz(i, static_cast<float>(i));
  }

  for (int i = 0; i < view.Size(); ++i) {
    EXPECT_THAT(points.AtUnsafe(i),
                IsApprox(eigenmath::Vector3f(static_cast<float>(3 * i + 0),
                                             static_cast<float>(3 * i + 1),
                                             static_cast<float>(3 * i + 2))));
  }
}

TEST(MultichannelCloud, PointCloudPoseSensor) {
  MultichannelCloudBuffer cloud(1, 1);
  EXPECT_THAT(cloud.PointCloudPoseSensor(), IsApprox(eigenmath::Pose3d()));

  eigenmath::Pose3d point_cloud_pose_sensor(
      eigenmath::Quaterniond(0.5, 0.5, 0.5, 0.5), eigenmath::Vector3d(1, 2, 3));
  cloud.SetPointCloudPoseSensor(point_cloud_pose_sensor);
  EXPECT_THAT(cloud.PointCloudPoseSensor(), IsApprox(point_cloud_pose_sensor));
}

TEST(MultichannelCloud, ViewCopyIsActuallyACopy) {
  MultichannelCloudBuffer buffer(1, 1);
  buffer.GetOrCreatePoints().At(0) = eigenmath::Vector3f(52, 53, 54);
  const auto cloud_view = ConstMultichannelCloudView(buffer.Proto());
  MultichannelCloudBuffer buffer_copy = cloud_view.Copy();
  // Make sure it's a copy. Mutate it and check that the original did not
  // change.
  buffer_copy.GetPoints().At(0) = eigenmath::Vector3f(1, 2, 3);
  EXPECT_EQ(buffer.GetPoints().At(0), eigenmath::Vector3f(52, 53, 54));
}

TEST(MultichannelCloud, ViewIsCopiedCorrectly) {
  MultichannelCloudBuffer buffer(1, 1);
  buffer.GetOrCreatePoints().At(0) = eigenmath::Vector3f(52, 53, 54);
  const auto cloud_view = ConstMultichannelCloudView(buffer.Proto());
  // Check that a point value was correctly copied.
  EXPECT_EQ(cloud_view.Copy().GetPoints().At(0),
            eigenmath::Vector3f(52, 53, 54));
}

}  // namespace

}  // namespace blue::mobility
