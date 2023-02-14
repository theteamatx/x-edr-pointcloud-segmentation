#include "googlex/proxy/object_properties/point_cloud/multichannel_cloud.h"

#include <string>

#include "googlex/proxy/conversions/conversions.h"
#include "googlex/proxy/module_system/core/message_header.proto.h"
#include "googlex/proxy/object_properties/point_cloud/cloud_proto_utils.h"

namespace blue::mobility {

RobotTime ConstMultichannelCloud::Timestamp() const {
  return RobotClock::FromNSec(const_proto_->header().timestamp());
}

void MultichannelCloud::SetTimestamp(const RobotTime& timestamp) {
  proto_->mutable_header()->set_timestamp(RobotClock::ToNSec(timestamp));
}

std::string ConstMultichannelCloud::FrameId() const {
  return const_proto_->header().frame_id();
}

void MultichannelCloud::SetFrameId(absl::string_view frame_id) {
  proto_->mutable_header()->set_frame_id(frame_id);
}

int64_t ConstMultichannelCloud::SequenceNumber() const {
  return const_proto_->header().sequence_number();
}

void MultichannelCloud::SetSequenceNumber(int64_t sequence_number) {
  proto_->mutable_header()->set_sequence_number(sequence_number);
}

eigenmath::Pose3d ConstMultichannelCloud::PointCloudPoseSensor() const {
  eigenmath::Pose3d point_cloud_pose_sensor;
  if (const_proto_->has_point_cloud_pose_sensor()) {
    point_cloud_pose_sensor =
        conversions::PoseFromProto(const_proto_->point_cloud_pose_sensor());
  }
  return point_cloud_pose_sensor;
}

void MultichannelCloud::SetPointCloudPoseSensor(
    const eigenmath::Pose3d& point_cloud_pose_sensor) {
  *proto_->mutable_point_cloud_pose_sensor() =
      conversions::ProtoFromPose(point_cloud_pose_sensor);
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

}  // namespace blue::mobility
