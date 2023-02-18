#include "pointcloud_segmentation/multichannel_cloud.h"

#include <string>

#include "eigenmath/conversions.h"
#include "pointcloud_segmentation/cloud_proto_utils.h"

namespace mobility {

eigenmath::Pose3d ConstMultichannelCloud::PointCloudPoseSensor() const {
  eigenmath::Pose3d point_cloud_pose_sensor;
  if (const_proto_->has_point_cloud_pose_sensor()) {
    point_cloud_pose_sensor = eigenmath::conversions::PoseFromProto(
        const_proto_->point_cloud_pose_sensor());
  }
  return point_cloud_pose_sensor;
}

void MultichannelCloud::SetPointCloudPoseSensor(
    const eigenmath::Pose3d& point_cloud_pose_sensor) {
  *proto_->mutable_point_cloud_pose_sensor() =
      eigenmath::conversions::ProtoFromPose(point_cloud_pose_sensor);
}

MultichannelCloudBuffer ConstMultichannelCloud::Transform(
    const eigenmath::Pose3d& new_pose_current) const {
  MultichannelCloudBuffer output(Rows(), Cols());
  *output.Proto() = *Proto();
  output.TransformInPlace(new_pose_current);
  return output;
}

MultichannelCloudBuffer ConstMultichannelCloud::Copy() const {
  return MultichannelCloudBuffer(Proto());
}

void MultichannelCloud::TransformInPlace(
    const eigenmath::Pose3d& new_pose_current) {
  const eigenmath::Pose3f& new_posef_current = new_pose_current.cast<float>();

  if (HasPoints()) {
    for (eigenmath::Vector3f& point : GetPoints()) {
      point = new_posef_current * point;
    }
  }

  if (HasNormals()) {
    for (eigenmath::Vector3f& normal : GetNormals()) {
      normal = new_posef_current.rotationMatrix() * normal;
    }
  }
}

#define DEFINE_ACCESSORS(name, field_name, type, proto_field_type)            \
  bool ConstMultichannelCloud::Has##name() const {                            \
    return const_proto_->field_name##_size() != 0;                            \
  }                                                                           \
                                                                              \
  void MultichannelCloud::Clear##name() { proto_->clear_##field_name(); }     \
                                                                              \
  CloudView<type> MultichannelCloud::GetOrCreate##name(                       \
      proto_field_type default_proto_value) {                                 \
    return GetOrCreateCloudView<type, proto_field_type>(                      \
        Rows(), Cols(), default_proto_value, proto_->mutable_##field_name()); \
  }                                                                           \
                                                                              \
  CloudView<type> MultichannelCloud::Get##name() {                            \
    return GetMutableCloudView<type, proto_field_type>(                       \
        Rows(), Cols(), proto_->mutable_##field_name());                      \
  }                                                                           \
                                                                              \
  CloudView<const type> ConstMultichannelCloud::Get##name() const {           \
    return GetConstCloudView<type, proto_field_type>(                         \
        Rows(), Cols(), &const_proto_->field_name());                         \
  }                                                                           \
                                                                              \
  void MultichannelCloud::Resize##name(int new_rows, int new_cols) {          \
    ResizeField<type, proto_field_type>(Rows(), Cols(), new_rows, new_cols,   \
                                        proto_->mutable_##field_name());      \
  }

DEFINE_ACCESSORS(Points, points_xyz, eigenmath::Vector3f, float);
DEFINE_ACCESSORS(Normals, normals_xyz, eigenmath::Vector3f, float);
DEFINE_ACCESSORS(Intensities, intensities, float, float);
DEFINE_ACCESSORS(Colors, colors_rgb_f, eigenmath::Vector3f, float);
DEFINE_ACCESSORS(CbrOrigins, cbr_origins, eigenmath::Vector3f, float);
DEFINE_ACCESSORS(ReturnPulseWidths, return_pulse_widths, eigenmath::Vector3f,
                 float);
DEFINE_ACCESSORS(ReturnIntensities, return_intensities, eigenmath::Vector3f,
                 float);
DEFINE_ACCESSORS(ReturnRanges, return_ranges, eigenmath::Vector3f, float);

#undef DEFINE_ACCESSORS

// Resizes all fields. When adding new fields, add them to this method.
void MultichannelCloud::Resize(int rows, int cols) {
  ResizePoints(rows, cols);
  ResizeNormals(rows, cols);
  ResizeIntensities(rows, cols);
  ResizeColors(rows, cols);
  ResizeCbrOrigins(rows, cols);
  ResizeReturnPulseWidths(rows, cols);
  ResizeReturnIntensities(rows, cols);
  ResizeReturnRanges(rows, cols);

  proto_->set_width(cols);
  proto_->set_height(rows);
}

MultichannelCloudBuffer::MultichannelCloudBuffer(int rows, int cols) {
  proto_buffer_.set_width(cols);
  proto_buffer_.set_height(rows);
  MultichannelCloud::Init(&proto_buffer_);
}

MultichannelCloudBuffer::MultichannelCloudBuffer(
    const MultichannelCloudProto* proto) {
  proto_buffer_.Clear();
  proto_buffer_ = *proto;
  MultichannelCloud::Init(&proto_buffer_);
}

MultichannelCloudBuffer::MultichannelCloudBuffer(
    const MultichannelCloudBuffer& other) {
  proto_buffer_ = other.proto_buffer_;
  MultichannelCloud::Init(&proto_buffer_);
}

MultichannelCloudBuffer& MultichannelCloudBuffer::operator=(
    const MultichannelCloudBuffer& other) {
  proto_buffer_.Clear();
  proto_buffer_ = other.proto_buffer_;
  MultichannelCloud::Init(&proto_buffer_);
  return *this;
}

MultichannelCloudView::MultichannelCloudView(MultichannelCloudProto* proto) {
  MultichannelCloud::Init(proto);
}

ConstMultichannelCloudView::ConstMultichannelCloudView(
    const MultichannelCloudProto* proto) {
  ConstMultichannelCloud::Init(proto);
}

#undef DEFINE_ACCESSORS

}  // namespace mobility
