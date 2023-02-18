#include "pointcloud_segmentation/planar_region.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace mobility {
namespace detail {

std::vector<Neighbor> CreateNeighborhood(
    NeighborhoodSelection neighborhood_selection, int rows, int cols) {
  std::vector<Neighbor> neighborhood;
  if (neighborhood_selection == kUse8Neighborhood) {
    neighborhood = {{-1, 0, -rows},       //
                    {-1, -1, -rows - 1},  //
                    {0, -1, -1},          //
                    {1, -1, rows - 1},    //
                    {1, 0, rows},         //
                    {1, 1, rows + 1},     //
                    {0, 1, 1},            //
                    {-1, 1, -rows + 1}};
  } else {
    neighborhood = {{-1, 0, -rows},  //
                    {0, -1, -1},     //
                    {1, 0, rows},    //
                    {0, 1, 1}};
  }
  return neighborhood;
}

int FindNextBoundaryPointDirection(const Cloud<int>& labels,
                                   const int region_label,
                                   const std::vector<Neighbor>& directions,
                                   const int curr_x, const int curr_y,
                                   const int curr_idx,
                                   const int direction_idx) {
  for (int delta_idx = 1; delta_idx <= directions.size(); ++delta_idx) {
    const int new_direction_index =
        (direction_idx + delta_idx) % directions.size();
    const int x = curr_x + directions[new_direction_index].delta_x;
    const int y = curr_y + directions[new_direction_index].delta_y;
    const int index = curr_idx + directions[new_direction_index].delta_index;
    if (x >= 0 && x < labels.Cols() && y >= 0 && y < labels.Rows() &&
        labels.AtUnsafe(index) == region_label) {
      return new_direction_index;
    }
  }
  return -1;
}

int FindInitialPredecessorDirection(const Cloud<int>& labels,
                                    const int region_label,
                                    const std::vector<Neighbor>& directions,
                                    const int curr_x, const int curr_y,
                                    const int curr_idx) {
  for (int i = 0; i < directions.size(); ++i) {
    const int x = curr_x + directions[i].delta_x;
    const int y = curr_y + directions[i].delta_y;
    const int index = curr_idx + directions[i].delta_index;
    if (x >= 0 && x < labels.Cols() && y >= 0 && y < labels.Rows() &&
        labels.AtUnsafe(index) != region_label) {
      return i;
    }
  }
  return -1;
}

bool InBounds(const Cloud<int>& labels, const int idx, const Neighbor& n) {
  int x = (idx / labels.Rows()) + n.delta_x;
  int y = (idx % labels.Rows()) + n.delta_y;
  if (x >= 0 && x < labels.Cols() && y >= 0 && y < labels.Rows()) return true;
  return false;
}

bool CheckMinRowsAndCols(const std::vector<int>& indices, int rows, int cols,
                         int min_cols, int min_rows) {
  int min_x = cols;
  int max_x = 0;
  int min_y = rows;
  int max_y = 0;
  for (int index : indices) {
    int x = index / rows;
    int y = index % rows;
    min_x = std::min(x, min_x);
    max_x = std::max(x, max_x);
    min_y = std::min(y, min_y);
    max_y = std::max(y, max_y);
  }
  return ((max_x - min_x) > min_cols) && ((max_y - min_y) > min_rows);
}

}  // namespace detail

}  // namespace mobility
