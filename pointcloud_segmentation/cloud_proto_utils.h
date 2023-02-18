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

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_PROTO_UTILS_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_PROTO_UTILS_H_

#include <algorithm>
#include <type_traits>

#include "google/protobuf/repeated_field.h"
#include "pointcloud_segmentation/cloud.h"

namespace mobility {

// These helpers are called by the macros below for easier compile checking and
// better syntax highlighting (and to keep the macros simple).
template <typename T, typename ProtoFieldT>
struct AssertCloudTypeCompatibility {
  static_assert(sizeof(T) % sizeof(ProtoFieldT) == 0,
                "Proto field type does not fit into requested type");
  static_assert(alignof(T) == alignof(ProtoFieldT),
                "Requested type may not fall on its alignment boundary");
  static_assert(std::is_standard_layout<T>::value,
                "Requested type is not standard layout");
};

// Memorymaps a const view onto a proto repeated field.
template <typename T, typename ProtoFieldT>
CloudView<const T> GetConstCloudView(
    int rows, int cols,
    const ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  AssertCloudTypeCompatibility<const T, ProtoFieldT>();
  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);

  if (proto_data->empty() || proto_data->size() < field_width * rows * cols) {
    return CloudView<const T>(nullptr, 0, 0);
  }

  const T* data = reinterpret_cast<const T*>(proto_data->data());
  return CloudView<const T>(data, rows, cols);
}

// Memorymaps a const view as a single row onto a proto repeated field.
template <typename T, typename ProtoFieldT>
CloudView<const T> GetConstCloudViewSingleRow(
    const ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);
  CHECK_EQ(proto_data->size() % field_width, 0);
  return GetConstCloudView<T>(1, proto_data->size() / field_width, proto_data);
}

// Memorymaps a mutable view onto a proto repeated field.
template <typename T, typename ProtoFieldT>
CloudView<T> GetMutableCloudView(
    int rows, int cols,
    ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  AssertCloudTypeCompatibility<T, ProtoFieldT>();
  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);

  if (proto_data->empty() || proto_data->size() < field_width * rows * cols) {
    return CloudView<T>(nullptr, 0, 0);
  }

  T* data = reinterpret_cast<T*>(proto_data->mutable_data());
  return CloudView<T>(data, rows, cols);
}

// Memorymaps a mutable view as a single row onto a proto repeated field.
template <typename T, typename ProtoFieldT>
CloudView<const T> GetMutableCloudViewSingleRow(
    ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);
  CHECK_EQ(proto_data->size() % field_width, 0);
  return GetMutableCloudView<T>(1, proto_data->size() / field_width,
                                proto_data);
}

// Possibly allocates memory to build a repeated field, then returns a mutable
// memorymapped cloud view.
template <typename T, typename ProtoFieldT>
CloudView<T> GetOrCreateCloudView(
    int rows, int cols, ProtoFieldT default_value,
    ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  AssertCloudTypeCompatibility<T, ProtoFieldT>();
  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);

  const int cloud_scalar_size = rows * cols * field_width;
  if (proto_data->size() != cloud_scalar_size) {
    proto_data->Resize(cloud_scalar_size, default_value);
  }

  return GetMutableCloudView<T, ProtoFieldT>(rows, cols, proto_data);
}

// Resizes a field and copies elements over according to row/column access.
template <typename T, typename ProtoFieldT>
void ResizeField(int orig_rows, int orig_cols, int new_rows, int new_cols,
                 ::google::protobuf::RepeatedField<ProtoFieldT>* proto_data) {
  if (proto_data->empty()) {
    return;
  }

  CloudBuffer<T> orig_copy =
      GetConstCloudView<T, ProtoFieldT>(orig_rows, orig_cols, proto_data)
          .Copy();

  constexpr int field_width = sizeof(T) / sizeof(ProtoFieldT);
  proto_data->Resize(new_rows * new_cols * field_width, ProtoFieldT());

  CloudView<T> new_cloud =
      GetMutableCloudView<T, ProtoFieldT>(new_rows, new_cols, proto_data);

  const int overlap_cols = std::min(orig_cols, new_cols);
  const int overlap_rows = std::min(orig_rows, new_rows);
  for (int col = 0; col < overlap_cols; ++col) {
    for (int row = 0; row < overlap_rows; ++row) {
      new_cloud.AtUnsafe(row, col) = orig_copy.AtUnsafe(row, col);
    }
  }
}

}  // namespace mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_PROTO_UTILS_H_
