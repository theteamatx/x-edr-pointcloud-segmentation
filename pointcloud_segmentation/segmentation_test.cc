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

#include "pointcloud_segmentation/segmentation.h"

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/planar_region.h"

namespace mobility {

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

}  // namespace mobility
