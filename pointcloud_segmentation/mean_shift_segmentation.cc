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

#include "pointcloud_segmentation/mean_shift_segmentation.h"

#include <cmath>

namespace mobility {

internal::PointIndex::PointIndex(int row_in, int col_in) {
  x() = row_in;
  y() = col_in;
}

internal::PointIndex::PointIndex(float row_in, float col_in) {
  x() = row_in;
  y() = col_in;
}

int internal::PointIndex::ColIndex() const { return std::round(Col()); }

int internal::PointIndex::RowIndex() const { return std::round(Row()); }

}  // namespace mobility
