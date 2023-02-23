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

#include "pointcloud_segmentation/plane_estimator.h"

#include <cmath>
#include <utility>
#include <vector>

#include "eigenmath/matchers.h"
#include "eigenmath/pose2.h"
#include "eigenmath/pose3.h"
#include "eigenmath/types.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pointcloud_segmentation/multichannel_cloud.h"

namespace mobility {
namespace {

// The normal error has numerical non-determinism issue due to the accumulator
// stores the squrated floats, thus the default error tolerance fails the test.
constexpr double kMaxErrorTolerance = 1e-4;

constexpr float kEpsilon = 1.0e-5f;

TEST(PlaneEstimatorTest, PlaneNormal) {
  // Generate test points on the test plane.
  eigenmath::Vector3f expect_normal(3.001, 2.0001, 1.02);
  expect_normal.normalize();
  float expect_off_set = 10;
  std::vector<eigenmath::Vector3f> points;
  for (int ii = -100; ii < 100; ++ii) {
    for (int jj = -100; jj < 100; ++jj) {
      float x = ii / 10.0;
      float y = jj / 10.0;
      eigenmath::Vector3f point(
          x, y,
          (expect_off_set - expect_normal.x() * x - expect_normal.y() * y) /
              expect_normal.z());
      points.emplace_back(std::move(point));
    }
  }

  PlaneEstimator plane_estimator1;
  PlaneEstimator plane_estimator2;
  for (int i = 0; i < points.size(); ++i) {
    plane_estimator1.AddPoint(points[i]);
    plane_estimator2.AddPoint(points[points.size() - i - 1]);
  }
  ASSERT_TRUE(plane_estimator1.PlaneValid());
  EXPECT_THAT(plane_estimator1.Plane().normal(),
              eigenmath::testing::IsApprox(expect_normal.cast<float>(),
                                           kMaxErrorTolerance));

  ASSERT_TRUE(plane_estimator2.PlaneValid());
  EXPECT_THAT(
      plane_estimator1.Plane().normal(),
      eigenmath::testing::IsApprox(
          plane_estimator2.Plane().normal().cast<float>(), kMaxErrorTolerance));
}

TEST(TestPlaneEstimator, CornerCases) {
  ::mobility::PlaneEstimator plane_estimator;
  ::eigenmath::Vector3f point = {1.0f, 2.0f, 3.0f};

  plane_estimator.SetNormalOrientation({0.1f, 0.1f, 0.98f});

  // First add the same point three times to see if the plane remains invalid.
  EXPECT_FALSE(plane_estimator.PlaneValid());
  plane_estimator.AddPoint(point);
  EXPECT_FALSE(plane_estimator.PlaneValid());
  EXPECT_EQ(plane_estimator.Curvature(), 0.0f);
  EXPECT_NEAR(plane_estimator.Centroid().x(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 2.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 3.0f, kEpsilon);
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().x()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().y()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().z()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().offset()));
  plane_estimator.AddPoint(point);
  EXPECT_FALSE(plane_estimator.PlaneValid());
  plane_estimator.AddPoint(point);
  EXPECT_FALSE(plane_estimator.PlaneValid());
  EXPECT_EQ(plane_estimator.Curvature(), 0.0f);
  EXPECT_NEAR(plane_estimator.Centroid().x(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 2.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 3.0f, kEpsilon);
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().x()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().y()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().z()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().offset()));
  EXPECT_NEAR(plane_estimator.Plane().normal().x(), 0.1f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().y(), 0.1f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().z(), 0.98f, kEpsilon);

  // Add a point so that the points form a line.
  point.x() = 0.0f;
  plane_estimator.AddPoint(point);
  EXPECT_FALSE(plane_estimator.PlaneValid());
  EXPECT_NEAR(plane_estimator.Centroid().x(), 0.75f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 2.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 3.0f, kEpsilon);
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().x()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().y()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().z()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().offset()));

  // Add another point so that we have a plane now.
  point.y() = 0.0f;
  plane_estimator.AddPoint(point);
  EXPECT_TRUE(plane_estimator.PlaneValid());
  EXPECT_NEAR(plane_estimator.Centroid().x(), 0.6f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 1.6f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 3.0f, kEpsilon);
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().x()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().y()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().normal().z()));
  EXPECT_FALSE(std::isnan(plane_estimator.Plane().offset()));
  EXPECT_NEAR(plane_estimator.Plane().normal().x(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().y(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().offset(), -3.0f, kEpsilon);

  // Test setting (flipping here) the normal orientation for a valid plane.
  plane_estimator.SetNormalOrientation({0.1f, 0.1f, -0.98f});
  EXPECT_NEAR(plane_estimator.Plane().normal().x(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().y(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().z(), -1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().offset(), 3.0f, kEpsilon);
}

TEST(TestPlaneEstimator, NoisyPlaneEstimation) {
  ::mobility::PlaneEstimator plane_estimator;
  plane_estimator.SetNormalOrientation({0.1f, 0.1f, 0.98f});
  plane_estimator.AddPoint({0.0f, 0.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 0.0f, 0.9f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 0.0f, 1.2f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 0.0f, 0.8f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 1.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 1.0f, 0.9f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 1.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 1.0f, 0.9f}, 1.0f);

  EXPECT_TRUE(plane_estimator.PlaneValid());
  EXPECT_NEAR(plane_estimator.Centroid().x(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().x(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().y(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().offset(), -1.0f, kEpsilon);
}

TEST(TestPlaneEstimator, WeightedPlaneEstimation) {
  constexpr float kNoisyEpsilon = 0.04f;
  ::mobility::PlaneEstimator plane_estimator;
  plane_estimator.SetNormalOrientation({0.1f, 0.1f, 0.98f});
  plane_estimator.AddPoint({0.0f, 0.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 0.0f, 0.9f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 0.0f, 1.2f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 0.0f, 0.8f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 1.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({1.0f, 1.0f, 0.9f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 1.0f, 1.1f}, 1.0f);
  plane_estimator.AddPoint({0.0f, 1.0f, 0.9f}, 1.0f);
  // Add noisy points with low weight
  plane_estimator.AddPoint({4.0f, 6.0f, 7.0f}, 0.001f);
  plane_estimator.AddPoint({-6.0f, 5.0f, 8.0f}, 0.001f);
  EXPECT_TRUE(plane_estimator.PlaneValid());
  EXPECT_NEAR(plane_estimator.Centroid().x(), 0.5f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().y(), 0.5f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Centroid().z(), 1.0f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().x(), 0.0f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().y(), 0.0f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().normal().z(), 1.0f, kNoisyEpsilon);
  EXPECT_NEAR(plane_estimator.Plane().offset(), -1.0f, kNoisyEpsilon);
}

TEST(TestPlaneEstimator, AdvancedPlaneMerging) {
  ::mobility::PlaneEstimator plane_estimator1, plane_estimator2;
  plane_estimator1.SetNormalOrientation({0.1f, 0.1f, 0.98f});

  ::mobility::MultichannelCloudBuffer cloud(1, 8);
  ::mobility::CloudView<::eigenmath::Vector3f> points =
      cloud.GetOrCreatePoints();

  points.AtUnsafe(0) = {0.0f, 0.0f, 1.1f};
  points.AtUnsafe(1) = {0.0f, 0.0f, 0.9f};
  points.AtUnsafe(2) = {1.0f, 0.0f, 1.2f};
  points.AtUnsafe(3) = {1.0f, 0.0f, 0.8f};
  points.AtUnsafe(4) = {1.0f, 1.0f, 1.1f};
  points.AtUnsafe(5) = {1.0f, 1.0f, 0.9f};
  points.AtUnsafe(6) = {0.0f, 1.0f, 1.1f};
  points.AtUnsafe(7) = {0.0f, 1.0f, 0.9f};

  plane_estimator1.AddPoints(points);
  EXPECT_TRUE(plane_estimator1.PlaneValid());
  EXPECT_NEAR(plane_estimator1.Centroid().x(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Centroid().y(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Centroid().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().x(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().y(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().offset(), -1.0f, kEpsilon);

  ::eigenmath::Pose3d frame2_pose_frame1(
      Eigen::AngleAxisd(M_PI / 2.0, ::eigenmath::Vector3d(1.0f, 0.0f, 0.0f))
          .toRotationMatrix(),
      ::eigenmath::Vector3d(1.0f, 1.0f, 1.0f));

  cloud.TransformInPlace(frame2_pose_frame1);

  plane_estimator2.SetNormalOrientation({0.1f, -0.98f, 0.1f});
  plane_estimator2.AddPoints(points);
  ::eigenmath::Vector3f normal2 =
      frame2_pose_frame1.rotationMatrix().cast<float>() *
      plane_estimator1.Plane().normal();
  ::eigenmath::Vector3f centroid2 =
      frame2_pose_frame1.cast<float>() * plane_estimator1.Centroid();
  EXPECT_TRUE(plane_estimator2.PlaneValid());
  EXPECT_NEAR(plane_estimator2.Centroid().x(), centroid2.x(), kEpsilon);
  EXPECT_NEAR(plane_estimator2.Centroid().y(), centroid2.y(), kEpsilon);
  EXPECT_NEAR(plane_estimator2.Centroid().z(), centroid2.z(), kEpsilon);
  EXPECT_NEAR(plane_estimator2.Plane().normal().x(), normal2.x(), kEpsilon);
  EXPECT_NEAR(plane_estimator2.Plane().normal().y(), normal2.y(), kEpsilon);
  EXPECT_NEAR(plane_estimator2.Plane().normal().z(), normal2.z(), kEpsilon);

  Eigen::Matrix4d frame1_pose_frame2 =
      frame2_pose_frame1.inverse().matrix().cast<double>();
  plane_estimator1.Merge(plane_estimator2,
                         ::eigenmath::Pose3d(frame1_pose_frame2));
  EXPECT_TRUE(plane_estimator1.PlaneValid());
  EXPECT_NEAR(plane_estimator1.Centroid().x(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Centroid().y(), 0.5f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Centroid().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().x(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().y(), 0.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().normal().z(), 1.0f, kEpsilon);
  EXPECT_NEAR(plane_estimator1.Plane().offset(), -1.0f, kEpsilon);
}

}  // namespace
}  // namespace mobility
