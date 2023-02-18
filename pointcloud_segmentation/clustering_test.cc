#include <vector>

#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/cluster_region.h"
#include "pointcloud_segmentation/segmentation.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// The generated cluster id of each point should look like (kSize=10):
//  1  1  1  1 2 3 3 3 3  4
//  1  1  1  1 2 3 3 3 4  4
//  1  1  1  1 2 3 3 4 4  5
//  1  1  1  1 2 3 4 4 5  5
// -1 -1 -1 -1 2 4 4 5 5  5
//  6  6  6  6 2 4 5 5 5  5
//  6  6  6  6 2 5 5 5 -1 5
//  6  6  6  6 2 5 5 5 -1 5
//  6  6  6  6 2 5 5 5 -1 5
//  6  6  6  6 2 5 5 5 5  5
// This test data contains
// 1) regular cluster
// 2) curved line
// 3) small cluster
// 4) straight line
// 5) two clusters that are close in the 3D world but disconnect by a line
// 6) faraway points and roof point

namespace blue::mobility {

namespace {
constexpr int kSize = 10;
CloudBuffer<eigenmath::Vector3f> points(kSize, kSize);
CloudBuffer<int> labels(kSize, kSize);
std::vector<int> seed_point_indices(kSize* kSize);
}  // namespace

void PointCloudData() {
  for (int col = 0; col < points.Cols(); ++col) {
    for (int row = 0; row < points.Rows(); ++row) {
      labels.AtUnsafe(row, col) = kUnlabeled;
      int index = points.LinearizeIndex(row, col);
      seed_point_indices[kSize * kSize - 1 - index] =
          points.LinearizeIndex(row, col);

      auto& point = points.AtUnsafe(row, col);
      if (col < kSize / 2 - 1) {
        if (row < kSize / 2 - 1) {
          // regular cluster
          point.x() = 0.05f * col + 2.0f;
          point.y() = 0.05f * row + 2.0f;
          point.z() = 2.0f;
        } else if (row > kSize / 2 - 1) {
          // regular cluster
          point.x() = 0.05f * col;
          point.y() = 0.05f * row + 2.0f;
          point.z() = 0.0f;
        } else {
          point.x() = 0.05f * col;
          point.y() = 0.05f * row + 2.0f;
          point.z() = 1.0f;
        }
        continue;
      }

      // A Single line
      if (col == kSize / 2 - 1) {
        point.x() = 0.1f * col;
        point.y() = 0.0f;
        point.z() = 0.1f * row;
        continue;
      }

      // The left is a big cluster that is disconnected by a curved line
      if (row + col == kSize || row + col == kSize - 1) {
        // curved line
        point.x() = 0.05f * col + 5.0f;
        point.y() = -0.05f * row - 2.0f;
        point.z() = 2.0f - 0.05f * row;
      } else if (row == kSize - 2 && col == kSize - 2) {
        // A roof point
        point.x() = 0.05f * col + 2.0f;
        point.y() = -0.05f * row - 2.0f;
        point.z() = 4.0f;
      } else if ((row == kSize - 4 || row == kSize - 3) && col == kSize - 2) {
        // A outlier point (>10 meter)
        point.x() = 6.0f;
        point.y() = 8.0f;
        point.z() = 2.0f;
      } else {
        // A big cluster devided by the curved line
        point.x() = 0.05f * col + 2.0f;
        point.y() = -0.05f * row - 2.0f;
        point.z() = 2.0f - 0.05f * row;
      }
    }
  }
}

TEST(CLUSTER_POINT_CLOUD_TEST, SEGMENT_CLUSTER) {
  PointCloudData();

  std::vector<ClusterRegion<eigenmath::Vector3f>> cluster_regions;

  // Suppose we have 1 region detected in plane_segmentation
  const int number_planar_region = 1;
  ClusterRegion<eigenmath::Vector3f>::InputData& input_data(points);
  SegmentRegions(input_data, seed_point_indices,
                 ClusterRegion<eigenmath::Vector3f>::Config(), &cluster_regions,
                 &labels, number_planar_region);

  std::vector<std::vector<int>> clusters_check = {
      {1, 1, 1, 1, 3, 4, 4, 4, 4, 5},      //
      {1, 1, 1, 1, 3, 4, 4, 4, 5, 5},      //
      {1, 1, 1, 1, 3, 4, 4, 5, 5, 6},      //
      {1, 1, 1, 1, 3, 4, 5, 5, 6, 6},      //
      {-1, -1, -1, -1, 3, 5, 5, 6, 6, 6},  //
      {2, 2, 2, 2, 3, 5, 6, 6, 6, 6},      //
      {2, 2, 2, 2, 3, 6, 6, 6, -1, 6},     //
      {2, 2, 2, 2, 3, 6, 6, 6, -1, 6},     //
      {2, 2, 2, 2, 3, 6, 6, 6, -1, 6},     //
      {2, 2, 2, 2, 3, 6, 6, 6, 6, 6}};

  for (int r = 0; r < kSize; ++r) {
    for (int c = 0; c < kSize; ++c) {
      int id = labels.AtUnsafe(r, c);
      EXPECT_EQ(id, clusters_check[r][c]) << "r, c = " << r << ", " << c;
    }
  }

  std::vector<int> point_number_each_cluster{16, 20, 10, 10, 10, 27};
  EXPECT_EQ(cluster_regions.size(), point_number_each_cluster.size());
  for (int i = 0; i < cluster_regions.size(); ++i) {
    EXPECT_EQ(cluster_regions[i].inlier_indices().size(),
              point_number_each_cluster[i]);
  }
}

}  // namespace blue::mobility

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
