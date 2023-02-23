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

#include <limits>
#include <vector>

#include "absl/log/check.h"

namespace mobility {

PlaneEstimator::PlaneEstimator() { Clear(); }

PlaneEstimator::PlaneEstimator(const PlaneEstimatorProto& proto)
    : plane_up_to_date_(false),
      plane_valid_(false),
      cumulative_centroid_(proto.cumulative_centroid(0),
                           proto.cumulative_centroid(1),
                           proto.cumulative_centroid(2)),
      cumulative_weights_(proto.cumulative_weights()) {
  accumulator_[0] = proto.covariance_accumulator(0);
  accumulator_[1] = proto.covariance_accumulator(1);
  accumulator_[2] = proto.covariance_accumulator(2);
  accumulator_[3] = proto.covariance_accumulator(3);
  accumulator_[4] = proto.covariance_accumulator(4);
  accumulator_[5] = proto.covariance_accumulator(5);
  plane_.normal().x() = proto.normal(0);
  plane_.normal().y() = proto.normal(1);
  plane_.normal().z() = proto.normal(2);
}

void PlaneEstimator::Clear() {
  cumulative_weights_ = 0.0f;
  plane_up_to_date_ = true;
  plane_valid_ = false;
  cumulative_centroid_.setZero();
  accumulator_.setZero();
  plane_.coeffs() = {1.0f, 0.0f, 0.0f, 0.0f};
}

void PlaneEstimator::AddPoint(const eigenmath::Vector3f& point) {
  accumulator_[0] += point.x() * point.x();
  accumulator_[1] += point.x() * point.y();
  accumulator_[2] += point.x() * point.z();
  accumulator_[3] += point.y() * point.y();
  accumulator_[4] += point.y() * point.z();
  accumulator_[5] += point.z() * point.z();
  cumulative_centroid_ += point;
  cumulative_weights_ += 1.0f;
  plane_up_to_date_ = false;
}

void PlaneEstimator::AddPoint(const eigenmath::Vector3f& point, float weight) {
  accumulator_[0] += point.x() * point.x() * weight;
  accumulator_[1] += point.x() * point.y() * weight;
  accumulator_[2] += point.x() * point.z() * weight;
  accumulator_[3] += point.y() * point.y() * weight;
  accumulator_[4] += point.y() * point.z() * weight;
  accumulator_[5] += point.z() * point.z() * weight;
  cumulative_centroid_ += point * weight;
  cumulative_weights_ += weight;
  plane_up_to_date_ = false;
}

void PlaneEstimator::AddPoints(const std::vector<eigenmath::Vector3f>& points) {
  for (const auto& point : points) {
    AddPoint(point);
  }
}

const eigenmath::Plane3f& PlaneEstimator::Plane() const {
  if (!plane_up_to_date_) {
    ComputePlane();
  }
  return plane_;
}

bool PlaneEstimator::PlaneValid() const {
  if (!plane_up_to_date_) {
    ComputePlane();
  }
  return plane_valid_;
}

const eigenmath::Vector3f& PlaneEstimator::Centroid() const {
  if (!plane_up_to_date_) {
    ComputePlane();
  }
  return centroid_;
}

float PlaneEstimator::Curvature() const {
  if (!plane_up_to_date_) {
    ComputePlane();
  }
  return curvature_;
}

void PlaneEstimator::SetNormalOrientation(
    const eigenmath::Vector3f& normal_orientation_hint) {
  if (PlaneValid()) {
    // Flip plane normal, if it is not pointing in the direction of the
    // orientation hint.
    if (plane_.normal().dot(normal_orientation_hint) < 0.0f) {
      plane_.coeffs() *= -1.0f;
    }
  } else {
    // Initialize plane normal with the orientation hint, so that the normal
    // direction is preserved in ComputePlane.
    plane_.normal() = normal_orientation_hint;
  }
}

void PlaneEstimator::Merge(const PlaneEstimator& other) {
  accumulator_ += other.accumulator_;
  cumulative_centroid_ += other.cumulative_centroid_;
  cumulative_weights_ += other.cumulative_weights_;
  plane_up_to_date_ = false;
}

void PlaneEstimator::Merge(const PlaneEstimator& other,
                           const eigenmath::Pose3d& this_pose_other) {
  PlaneEstimator other_transformed(other);
  other_transformed.Transform(this_pose_other);
  Merge(other_transformed);
}

void PlaneEstimator::Transform(const eigenmath::Pose3d& new_pose_current) {
  CHECK_GT(cumulative_weights_, 0.0f) << "No points added to plane estimator?";
  if (!plane_up_to_date_) {
    centroid_ = cumulative_centroid_ / cumulative_weights_;
  }
  // Rotate the accumulated covariance.
  eigenmath::Vector6f accu = accumulator_ / cumulative_weights_;
  eigenmath::Matrix3f covariance_matrix = eigenmath::Matrix3f::Zero();
  covariance_matrix.coeffRef(0) = accu[0] - centroid_.x() * centroid_.x();
  covariance_matrix.coeffRef(1) = accu[1] - centroid_.x() * centroid_.y();
  covariance_matrix.coeffRef(2) = accu[2] - centroid_.x() * centroid_.z();
  covariance_matrix.coeffRef(4) = accu[3] - centroid_.y() * centroid_.y();
  covariance_matrix.coeffRef(5) = accu[4] - centroid_.y() * centroid_.z();
  covariance_matrix.coeffRef(8) = accu[5] - centroid_.z() * centroid_.z();
  covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
  covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
  covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
  // Do the covariance matrix rotation.
  eigenmath::Matrix3f rotation =
      new_pose_current.rotationMatrix().cast<float>();
  eigenmath::Matrix3f covariance_matrix_transformed =
      rotation * covariance_matrix * rotation.transpose();
  // Transform cumulative centroid to our new coordinate frame.
  eigenmath::Vector3f new_centroid = new_pose_current.cast<float>() * centroid_;
  // Write back rotated covariance matrix to our transformed plane estimator.
  accumulator_[0] = covariance_matrix_transformed.coeffRef(0) +
                    new_centroid.x() * new_centroid.x();
  accumulator_[1] = covariance_matrix_transformed.coeffRef(1) +
                    new_centroid.x() * new_centroid.y();
  accumulator_[2] = covariance_matrix_transformed.coeffRef(2) +
                    new_centroid.x() * new_centroid.z();
  accumulator_[3] = covariance_matrix_transformed.coeffRef(4) +
                    new_centroid.y() * new_centroid.y();
  accumulator_[4] = covariance_matrix_transformed.coeffRef(5) +
                    new_centroid.y() * new_centroid.z();
  accumulator_[5] = covariance_matrix_transformed.coeffRef(8) +
                    new_centroid.z() * new_centroid.z();
  accumulator_ *= cumulative_weights_;
  cumulative_centroid_ = new_centroid * cumulative_weights_;
  plane_up_to_date_ = false;
}

void PlaneEstimator::ComputePlane() const {
  CHECK_GT(cumulative_weights_, 0.0f) << "No points added to plane estimator?";
  plane_up_to_date_ = true;
  centroid_ = cumulative_centroid_ / cumulative_weights_;

  eigenmath::Vector6f accu = accumulator_ / cumulative_weights_;
  eigenmath::Matrix3f covariance_matrix = eigenmath::Matrix3f::Zero();
  covariance_matrix.coeffRef(0) = accu[0] - centroid_.x() * centroid_.x();
  covariance_matrix.coeffRef(1) = accu[1] - centroid_.x() * centroid_.y();
  covariance_matrix.coeffRef(2) = accu[2] - centroid_.x() * centroid_.z();
  covariance_matrix.coeffRef(4) = accu[3] - centroid_.y() * centroid_.y();
  covariance_matrix.coeffRef(5) = accu[4] - centroid_.y() * centroid_.z();
  covariance_matrix.coeffRef(8) = accu[5] - centroid_.z() * centroid_.z();
  covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
  covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
  covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);

  // Extract the smallest eigenvalue and its eigenvector.
  Eigen::SelfAdjointEigenSolver<eigenmath::Matrix3f> eigensolver;
  eigensolver.computeDirect(covariance_matrix);
  if (eigensolver.info() == Eigen::ComputationInfo::Success &&
      eigensolver.eigenvalues()[1] > std::numeric_limits<float>::min()) {
    float smallest_eigen_value = eigensolver.eigenvalues()(0);
    eigenmath::Vector3f eigen_vector = eigensolver.eigenvectors().col(0);
    // Preserve the old normal orientation.
    if (plane_.normal().dot(eigen_vector) > 0.0f) {
      plane_ = eigenmath::Plane3f(eigen_vector, centroid_);
    } else {
      plane_ = eigenmath::Plane3f(-eigen_vector, centroid_);
    }

    float covariance_matrix_trace = covariance_matrix.trace();
    if (covariance_matrix_trace > smallest_eigen_value &&
        smallest_eigen_value > std::numeric_limits<float>::min()) {
      // Compute the curvature surface change
      curvature_ = std::abs(smallest_eigen_value / covariance_matrix_trace);
    } else {
      curvature_ = 0.0f;
    }
    plane_valid_ = true;
  } else {
    curvature_ = 0.0f;
    plane_ = eigenmath::Plane3f(plane_.normal(), centroid_);
    plane_valid_ = false;
  }
}

void PlaneEstimator::ToProto(PlaneEstimatorProto* proto) const {
  proto->clear_covariance_accumulator();
  for (int i = 0; i < 6; ++i) {
    proto->add_covariance_accumulator(accumulator_[i]);
  }
  proto->clear_cumulative_centroid();
  for (int i = 0; i < 3; ++i) {
    proto->add_cumulative_centroid(cumulative_centroid_[i]);
  }
  proto->set_cumulative_weights(cumulative_weights_);
  proto->clear_normal();
  for (int i = 0; i < 3; ++i) {
    proto->add_normal(plane_.normal()[i]);
  }
}

}  // namespace mobility
