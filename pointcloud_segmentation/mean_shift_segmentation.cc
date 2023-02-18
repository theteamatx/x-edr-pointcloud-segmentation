#include "pointcloud_segmentation/mean_shift_segmentation.h"

#include <cmath>

namespace blue::mobility {

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

}  // namespace blue::mobility
