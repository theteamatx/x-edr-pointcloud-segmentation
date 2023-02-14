#include "googlex/proxy/object_properties/point_cloud/segmentation.h"

namespace blue::mobility {

CloudBuffer<int> PrepareUnlabeledLabels(
    const Cloud<eigenmath::Vector3f>& point_cloud) {
  CloudBuffer<int> labels(point_cloud.Rows(), point_cloud.Cols());
  labels.Fill(kUnlabeled);
  return labels;
}

}  // namespace blue::mobility
