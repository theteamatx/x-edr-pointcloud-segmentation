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

#ifndef MOBILITY_POINT_CLOUD_MULTICHANNEL_CLOUD_H_
#define MOBILITY_POINT_CLOUD_MULTICHANNEL_CLOUD_H_

#include <string>

#include "eigenmath/pose3.h"
#include "eigenmath/types.h"
#include "pointcloud_segmentation/cloud.h"
#include "pointcloud_segmentation/multichannel_cloud.pb.h"

namespace mobility {

class MultichannelCloudBuffer;

// A ConstMultichannelCloud represents immutable sensor data organized into
// a grid, associated with a 3d point per grid cell. For efficiency, it
// directly memorymaps a  protobuf object and provides a nice interface for
// C++ code, e.g., stereo cameras with RGB, 3d lidar with intensity.
class ConstMultichannelCloud {
 public:
  // Dimensions of the cloud.
  int Cols() const { return const_proto_->width(); }
  int Rows() const { return const_proto_->height(); }
  int Size() const { return Rows() * Cols(); }

  // Returns underlying proto.
  const MultichannelCloudProto* Proto() const { return const_proto_; }

  // Returns and sets the pose of the sensor with respect to FrameId().
  // Note: Cache this value, as it is converted from proto each time to stay
  // fresh.
  eigenmath::Pose3d PointCloudPoseSensor() const;

  // Transforms all transformable aspects of the cloud (e.g. points and
  // normals).
  MultichannelCloudBuffer Transform(
      const eigenmath::Pose3d& new_pose_current) const;

  // Creates a deep copy of this cloud.
  MultichannelCloudBuffer Copy() const;

  // Accessors for channels in the cloud.
  // Implementers note: When adding a field, add it to the .cc macros as well,
  // and update the Resize() and Transform() methods.
#define DECLARE_CONST_ACCESSORS(name, type, proto_field_type) \
 public:                                                      \
  bool Has##name() const;                                     \
  CloudView<const type> Get##name() const;

  // Accessors for xyz points of the point cloud.
  DECLARE_CONST_ACCESSORS(Points, eigenmath::Vector3f, float)

  // Accessors for per-point normals of the point cloud.
  DECLARE_CONST_ACCESSORS(Normals, eigenmath::Vector3f, float)

  // Accessors for per-point intensity of the point cloud.
  DECLARE_CONST_ACCESSORS(Intensities, float, float)

  // Accessors for per-point colors of the point cloud.
  DECLARE_CONST_ACCESSORS(Colors, eigenmath::Vector3f, float)

  // Accessors for per-point attributes of the point cloud:
  //   cbr_origins, pulse_widths, intensities, return ranges.
  DECLARE_CONST_ACCESSORS(CbrOrigins, eigenmath::Vector3f, float)
  DECLARE_CONST_ACCESSORS(ReturnPulseWidths, eigenmath::Vector3f, float)
  DECLARE_CONST_ACCESSORS(ReturnIntensities, eigenmath::Vector3f, float)
  DECLARE_CONST_ACCESSORS(ReturnRanges, eigenmath::Vector3f, float)
#undef DECLARE_CONST_ACCESSORS

 protected:
  // Protected so the only possible creation is through subclasses.
  ConstMultichannelCloud() {}
  ~ConstMultichannelCloud() {}
  void Init(const MultichannelCloudProto* proto) { const_proto_ = proto; }

 private:
  const MultichannelCloudProto* const_proto_;
};

// A MultichannelCloud represents sensor data organized into a grid, associated
// with a 3d point per grid cell. For efficiency, it directly memorymaps a
// protobuf object and provides a nice interface for C++ code.
// e.g. stereo cameras with RGB, 3d lidar with intensity
class MultichannelCloud : public ConstMultichannelCloud {
 public:
  // Returns underlying proto.
  using ConstMultichannelCloud::Proto;
  MultichannelCloudProto* Proto() { return proto_; }

  // Resizes all channels of the cloud.
  void Resize(int rows, int cols);

  // Returns and sets the pose of the sensor with respect to FrameId().
  // Note: Cache this value, as it is converted from proto each time to stay
  // fresh.
  void SetPointCloudPoseSensor(
      const eigenmath::Pose3d& point_cloud_pose_sensor);

  // Transforms all transformable aspects of the cloud (e.g. points and
  // normals).
  void TransformInPlace(const eigenmath::Pose3d& new_pose_current);

  // Accessors for channels in the cloud.
  // Implementers note: When adding a field, add it to the .cc macros as well,
  // and update the Resize() and Transform() methods.
#define DECLARE_ACCESSORS(name, type, proto_field_type)                    \
 public:                                                                   \
  void Clear##name();                                                      \
  CloudView<type> GetOrCreate##name(proto_field_type default_proto_value = \
                                        proto_field_type());               \
  CloudView<type> Get##name();                                             \
  using ConstMultichannelCloud::Get##name;                                 \
                                                                           \
 private:                                                                  \
  void Resize##name(int rows, int cols);

  // Accessors for xyz points of the point cloud.
  DECLARE_ACCESSORS(Points, eigenmath::Vector3f, float)

  // Accessors for per-point normals of the point cloud.
  DECLARE_ACCESSORS(Normals, eigenmath::Vector3f, float)

  // Accessors for per-point intensity of the point cloud.
  DECLARE_ACCESSORS(Intensities, float, float)

  // Accessors for per-point colors of the point cloud.
  DECLARE_ACCESSORS(Colors, eigenmath::Vector3f, float)

  // Accessors for per-point attributes of the point cloud:
  //   cbr_origins, pulse_widths, intensities, ranges.
  DECLARE_ACCESSORS(CbrOrigins, eigenmath::Vector3f, float)
  DECLARE_ACCESSORS(ReturnPulseWidths, eigenmath::Vector3f, float)
  DECLARE_ACCESSORS(ReturnIntensities, eigenmath::Vector3f, float)
  DECLARE_ACCESSORS(ReturnRanges, eigenmath::Vector3f, float)
#undef DECLARE_ACCESSORS

 protected:
  // Protected so the only possible creation is through subclasses.
  MultichannelCloud() {}
  ~MultichannelCloud() {}
  void Init(MultichannelCloudProto* proto) {
    ConstMultichannelCloud::Init(proto);
    proto_ = proto;
  }

 private:
  MultichannelCloudProto* proto_;
};

// MultichannelCloudBuffer owns its own memory.
class MultichannelCloudBuffer : public MultichannelCloud {
 public:
  // Constructs a new cloud buffer of the given dimensions.
  MultichannelCloudBuffer(int rows, int cols);

  // Constructs a new cloud buffer by copying the given proto.
  explicit MultichannelCloudBuffer(const MultichannelCloudProto* proto);

  // Explicit destructor.
  ~MultichannelCloudBuffer() {}

  // Copy constructor and assignment. Not default due to object slicing.
  MultichannelCloudBuffer(const MultichannelCloudBuffer& other);
  MultichannelCloudBuffer& operator=(const MultichannelCloudBuffer& other);

 private:
  MultichannelCloudProto proto_buffer_;
};

// MultichannelCloudView wraps an existing proto.
class MultichannelCloudView : public MultichannelCloud {
 public:
  // Constructs a new view on an existing proto.
  explicit MultichannelCloudView(MultichannelCloudProto* proto);

  // Explicit destructor.
  ~MultichannelCloudView() {}
};

// A const MultichannelCloudView wraps an existing proto.
class ConstMultichannelCloudView : public ConstMultichannelCloud {
 public:
  // Constructs a new view on an existing proto.
  ConstMultichannelCloudView(const MultichannelCloudProto* proto);

  // Explicit destructor.
  ~ConstMultichannelCloudView() {}
};

}  // namespace mobility

#endif  // MOBILITY_POINT_CLOUD_MULTICHANNEL_CLOUD_H_
