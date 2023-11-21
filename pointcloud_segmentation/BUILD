# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "multichannel_cloud_proto",
    srcs = [
        "multichannel_cloud.proto",
    ],
    deps = [
        "@x_edr_eigenmath//eigenmath:eigenmath_proto",
    ],
)

cc_proto_library(
    name = "multichannel_cloud_cc_proto",
    deps = [":multichannel_cloud_proto"],
)

proto_library(
    name = "detected_objects_proto",
    srcs = ["detected_objects.proto"],
    deps = [
        "@x_edr_eigenmath//eigenmath:eigenmath_proto",
    ],
)

cc_proto_library(
    name = "detected_objects_cc_proto",
    deps = [":detected_objects_proto"],
)

proto_library(
    name = "plane_estimator_proto",
    srcs = ["plane_estimator.proto"],
)

cc_proto_library(
    name = "plane_estimator_cc_proto",
    deps = [":plane_estimator_proto"],
)

proto_library(
    name = "plane_classification_config_proto",
    srcs = ["plane_classification_config.proto"],
)

cc_proto_library(
    name = "plane_classification_config_cc_proto",
    deps = [":plane_classification_config_proto"],
)

proto_library(
    name = "region_segmentation_config_proto",
    srcs = ["region_segmentation_config.proto"],
)

cc_proto_library(
    name = "region_segmentation_config_cc_proto",
    deps = [":region_segmentation_config_proto"],
)

cc_library(
    name = "cloud",
    hdrs = ["cloud.h"],
    deps = [
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "cloud_test",
    size = "small",
    srcs = ["cloud_test.cc"],
    deps = [
        ":cloud",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)

cc_library(
    name = "cloud_proto_utils",
    hdrs = ["cloud_proto_utils.h"],
    deps = [
        ":cloud",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "indices",
    hdrs = ["indices.h"],
)

cc_library(
    name = "segmentation",
    srcs = ["segmentation.cc"],
    hdrs = ["segmentation.h"],
    deps = [
        ":cloud",
        "@com_google_absl//absl/log",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "segmentation_test",
    srcs = [
        "segmentation_test.cc",
    ],
    deps = [
        ":cloud",
        ":planar_region",
        ":segmentation",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "multichannel_cloud",
    srcs = ["multichannel_cloud.cc"],
    hdrs = ["multichannel_cloud.h"],
    deps = [
        ":cloud",
        ":cloud_proto_utils",
        ":multichannel_cloud_cc_proto",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:conversions",
    ],
)

cc_test(
    name = "multichannel_cloud_test",
    size = "small",
    srcs = ["multichannel_cloud_test.cc"],
    deps = [
        ":multichannel_cloud",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@x_edr_eigenmath//eigenmath:eigenmath_cc_proto",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)

cc_library(
    name = "cluster_region",
    hdrs = ["cluster_region.h"],
    deps = [
        ":cloud",
        ":detected_objects_cc_proto",
        ":region_segmentation_config_cc_proto",
        ":segmentation",
    ],
)

cc_test(
    name = "clustering_test",
    srcs = [
        "clustering_test.cc",
    ],
    deps = [
        ":cloud",
        ":cluster_region",
        ":segmentation",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "plane_estimator",
    srcs = ["plane_estimator.cc"],
    hdrs = ["plane_estimator.h"],
    deps = [
        ":cloud",
        ":indices",
        ":plane_estimator_cc_proto",
        "@com_google_absl//absl/log",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "mean_shift_segmentation",
    srcs = ["mean_shift_segmentation.cc"],
    hdrs = ["mean_shift_segmentation.h"],
    deps = [
        ":cluster_region",
        ":region_segmentation_config_cc_proto",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "plane_estimator_test",
    srcs = ["plane_estimator_test.cc"],
    deps = [
        ":multichannel_cloud",
        ":plane_estimator",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)

cc_test(
    name = "algorithms_test",
    srcs = ["algorithms_test.cc"],
    deps = [
        ":algorithms",
        ":cloud",
        ":multichannel_cloud",
        ":multichannel_cloud_cc_proto",
        ":plane_estimator",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)

cc_library(
    name = "planar_region",
    srcs = ["planar_region.cc"],
    hdrs = [
        "planar_region.h",
        "semantic_types.h",
    ],
    deps = [
        ":algorithms",
        ":cloud",
        ":plane_estimator",
        ":region_segmentation_config_cc_proto",
        ":segmentation",
        "@com_google_absl//absl/container:flat_hash_set",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "plane_classification",
    srcs = ["plane_classification.cc"],
    hdrs = ["plane_classification.h"],
    deps = [
        ":planar_region",
        ":plane_classification_config_cc_proto",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_library(
    name = "algorithms",
    hdrs = ["algorithms.h"],
    deps = [
        ":indices",
        ":plane_estimator",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_mobility_collision//collision:collision_checking",
    ],
)

cc_library(
    name = "detected_objects",
    srcs = ["detected_objects.cc"],
    hdrs = ["detected_objects.h"],
    deps = [
        ":cloud",
        ":cloud_proto_utils",
        ":detected_objects_cc_proto",
        ":planar_region",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:conversions",
        "@x_edr_eigenmath//eigenmath:eigenmath_cc_proto",
    ],
)
