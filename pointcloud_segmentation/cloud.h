// Copyright 2023 Google LLC

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_H_
#define GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_H_

#include <algorithm>
#include <vector>

#include "base/logging.h"
#include "googlex/proxy/eigenmath/pose3.h"
#include "third_party/eigen3/Eigen/Core"
#include "third_party/eigen3/Eigen/StdVector"

namespace blue::mobility {

// Options for the Cloud type.
namespace cloud {
enum {
  ColMajor = 0,
  RowMajor = 0x1,
};

// Helper to linearize indices from row/column. Template resolution resolves
// which axis is major.
template <int Options>
typename std::enable_if<!(Options & cloud::RowMajor), int>::type LinearizeIndex(
    int row, int col, int rows, int cols) {
  return col * rows + row;
}

template <int Options>
typename std::enable_if<(Options & cloud::RowMajor), int>::type LinearizeIndex(
    int row, int col, int rows, int cols) {
  return row * cols + col;
}

// Helper to convert indices to row/column. Template resolution resolves which
// axis is major.
template <int Options>
typename std::enable_if<!(Options & cloud::RowMajor), void>::type IndexToRowCol(
    int index, int rows, int cols, int* row, int* col) {
  *row = index % rows;
  *col = index / rows;
}

template <int Options>
typename std::enable_if<(Options & cloud::RowMajor), void>::type IndexToRowCol(
    int index, int rows, int cols, int* row, int* col) {
  *col = index % cols;
  *row = index / cols;
}

}  // namespace cloud

// Forward decl for Cloud::Copy()
template <typename T, int Options>
class CloudBuffer;

// A cloud represents data organized into a grid. It can be xyz points, rgb
// data, per-point intensity, etc.
// RowMajor/ColumnMajor is determined by the Options parameter.
template <typename T, int Options = cloud::ColMajor>
class Cloud {
 public:
  using ValueT = T;
  using IteratorT = T*;
  using ConstIteratorT = const T*;
  using CopyT = CloudBuffer<typename std::remove_cv<T>::type, Options>;

  // Helper for users to linearize row/col into a linear index.
  int LinearizeIndex(int row, int col) const {
    return cloud::LinearizeIndex<Options>(row, col, rows_, cols_);
  }

  // Helper to convert indices to row/column. Template resolution resolves
  // which axis is major.
  void IndexToRowCol(int index, int* row, int* col) const {
    cloud::IndexToRowCol<Options>(index, rows_, cols_, row, col);
  }

  // Range-checked linear accessors
  T& At(int index) { return AtImpl(index); }
  const T& At(int index) const { return AtImpl(index); }

  // Range-checked row/col accessors.
  T& At(int row, int col) { return AtImpl(row, col); }
  const T& At(int row, int col) const { return AtImpl(row, col); }

  // Unsafe linear accessors.
  T& AtUnsafe(int index) { return AtUnsafeImpl(index); }
  const T& AtUnsafe(int index) const { return AtUnsafeImpl(index); }

  // Unsafe row/col accessors.
  T& AtUnsafe(int row, int col) { return AtUnsafeImpl(row, col); }
  const T& AtUnsafe(int row, int col) const { return AtUnsafeImpl(row, col); }

  // Fills grid with accessors.
  void Fill(const T& value) {
    for (T& element : *this) {
      element = value;
    }
  }

  // Dimensions.
  int Rows() const { return rows_; }
  int Cols() const { return cols_; }
  int Size() const { return rows_ * cols_; }
  int size() const { return Size(); }

  // Underlying data pointer.
  T* Data() { return data_; }
  const T* Data() const { return data_; }

  // Realizes a CloudBuffer with the same template paratmeters and copies this
  // Cloud's data.
  CopyT Copy() {
    CopyT buffer(Rows(), Cols());
    std::copy(begin(), end(), buffer.begin());
    return buffer;
  }

  // "Iterators" for range-based for loops.
  IteratorT begin() { return Data(); }
  ConstIteratorT begin() const { return Data(); }
  IteratorT end() { return Data() + Size(); }
  ConstIteratorT end() const { return Data() + Size(); }

 protected:
  // Protected to force users through subclass creation.
  Cloud() {}
  ~Cloud() {}

  // Initializes a new Cloud (called from subclasses).
  void Init(T* data, int rows, int cols) {
    data_ = data;
    rows_ = rows;
    cols_ = cols;
  }

 private:
  // Note: these *Impls are a clean way to handle const&/mutable& in the
  // accessors.
  T& AtImpl(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, rows_ * cols_);
    return AtUnsafeImpl(index);
  }

  T& AtImpl(int row, int col) const {
    CHECK_GE(row, 0);
    CHECK_LT(row, rows_);
    CHECK_GE(col, 0);
    CHECK_LT(col, cols_);
    return AtUnsafeImpl(row, col);
  }

  T& AtUnsafeImpl(int index) const { return data_[index]; }

  T& AtUnsafeImpl(int row, int col) const {
    return data_[LinearizeIndex(row, col)];
  }

  T* data_ = nullptr;
  int rows_ = 0;
  int cols_ = 0;
};

// A CloudBuffer owns its own memory.
template <typename T, int Options = cloud::ColMajor>
class CloudBuffer : public Cloud<T, Options> {
  static_assert(!std::is_const<T>::value,
                "CloudBuffer cannot store const T clouds");

 public:
  using BaseT = Cloud<T, Options>;

  // Constructs a new CloudBuffer of the given size (1 row, size cols).
  explicit CloudBuffer(int size) { Resize(size); }

  // Constructs a new CloudBuffer of the given size (rows, cols);
  CloudBuffer(int rows, int cols) { Resize(rows, cols); }

  // Copy constructor. Explicit so the base class data pointer doesn't point to
  // other's memory.
  CloudBuffer(const CloudBuffer<T, Options>& other) {
    data_ = other.data_;
    BaseT::Init(data_.data(), other.Rows(), other.Cols());
  }

  // Copy assignment operator.  Explicit so the base class data pointer doesn't
  // point to other's memory.
  CloudBuffer<T, Options>& operator=(const CloudBuffer<T, Options>& other) {
    data_ = other.data_;
    BaseT::Init(data_.data(), other.Rows(), other.Cols());
    return *this;
  }

  // Clears the point cloud and sets the size to (0, 0).
  void Clear() { Resize(0, 0); }

  // Resizes the point cloud to (1 row, size cols).
  void Resize(int size) { Resize(1, size); }

  // Resizes the point cloud to (rows, cols).
  void Resize(int rows, int cols) {
    data_.resize(rows * cols);
    BaseT::Init(data_.data(), rows, cols);
  }

  // Explicit destructor.
  ~CloudBuffer() {}

 private:
  // Note:: aligned_allocator is required for fixed-size vectorizable Eigen
  // types. These are disabled by default in our codebase, but it's the Eigen
  // default.
  std::vector<T, Eigen::aligned_allocator<T>> data_;
};

// A CloudView provides a memorymap onto an existing buffer.
template <typename T, int Options = 0>
class CloudView : public Cloud<T, Options> {
 public:
  using BaseT = Cloud<T, Options>;

  // Explicit cast so that CloudView<const T> can bind to const CloudView<T>&,
  // which provides consistency across algorithm interfaces. This is similar to
  operator const CloudView<typename std::remove_cv<T>::type, Options>&() {
    return *reinterpret_cast<
        CloudView<typename std::remove_cv<T>::type, Options>*>(this);
  }

  // Constructs a new CloudView into the given data buffer, with the given
  // dimensions. The buffer must extend to at least data + (rows * cols).
  CloudView(T* data, int rows, int cols) { BaseT::Init(data, rows, cols); }

  // Constructs a view into an existing Cloud.
  CloudView(Cloud<T, Options>* cloud) {
    BaseT::Init(cloud->Data(), cloud->Rows(), cloud->Cols());
  }

  // Explicit destructor.
  ~CloudView() {}
};

}  // namespace blue::mobility

#endif  // GOOGLEX_PROXY_OBJECT_PROPERTIES_POINT_CLOUD_CLOUD_H_
