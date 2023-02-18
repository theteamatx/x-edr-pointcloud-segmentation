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

#ifndef MOBILITY_POINT_CLOUD_INDICES_H_
#define MOBILITY_POINT_CLOUD_INDICES_H_

#include <type_traits>
#include <vector>

namespace mobility {

using Indices = std::vector<int>;
struct NoIndices {};

namespace detail {

// Helper struct to allow static_assert to always fail in the default templates
template <typename T>
struct assert_false : std::false_type {};

// Helper struct for partial template specialization that implements
// GetIndicesSize with specializations for Indices and NoIndices.
template <typename IndexedContainerT, typename IndicesT>
struct GetIndicesSizeHelper {
  static int GetSize(const IndexedContainerT& indexed_container,
                     const IndicesT& indices) {
    static_assert(assert_false<IndicesT>::value,
                  "Unsupported IndicesT template argument, must be either "
                  "Indices or NoIndices!");
  }
};

// Template specialization of GetIndex for IndicesT == NoIndices
template <typename IndexedContainerT>
struct GetIndicesSizeHelper<IndexedContainerT, NoIndices> {
  static int GetSize(const IndexedContainerT& indexed_container,
                     const NoIndices&) {
    return static_cast<int>(indexed_container.size());
  }
};

// Template specialization of GetIndex for IndicesT == Indices
template <typename IndexedContainerT>
struct GetIndicesSizeHelper<IndexedContainerT, Indices> {
  static int GetSize(const IndexedContainerT& indexed_container,
                     const Indices& indices) {
    return static_cast<int>(indices.size());
  }
};

}  // namespace detail

// Returns the i-th index from indices if IndicesT is of type Indices, or
// i, if IndicesT is of type NoIndices. All other IndicesT types yield a compile
// time error.
template <typename IndicesT>
inline int GetIndex(int i, const IndicesT& indices) {
  static_assert(detail::assert_false<IndicesT>::value,
                "Unsupported IndicesT template argument, must be either "
                "Indices or NoIndices!");
}

// Template specialization of GetIndex for IndicesT == NoIndices
template <>
inline int GetIndex<NoIndices>(int i, const NoIndices&) {
  return i;
}

// Template specialization of GetIndex for IndicesT == Indices
template <>
inline int GetIndex<Indices>(int i, const Indices& indices) {
  return indices[i];
}

// Returns the size of indices if IndicesT is of type Indices or the size of the
// indexed_container if IndicesT is of type NoIndices. All other IndicesT types
// yield a compile time error.
template <typename IndexedContainerT, typename IndicesT>
inline int GetIndicesSize(const IndexedContainerT& indexed_container,
                          const IndicesT& indices) {
  return detail::GetIndicesSizeHelper<IndexedContainerT, IndicesT>::GetSize(
      indexed_container, indices);
}

}  // namespace mobility

#endif  // MOBILITY_POINT_CLOUD_INDICES_H_
