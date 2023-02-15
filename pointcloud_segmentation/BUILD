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
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "multichannel_cloud_proto",
    srcs = [
        "multichannel_cloud.proto",
    ],
    cc_api_version = 2,
    deps = [
        "//googlex/proxy/blueproto:eigen_proto",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_proto",
        "//googlex/proxy/module_system/core:common_proto",
        "//third_party/robotics/logging:proto_options",
    ],
)

py_proto_library(
    name = "multichannel_cloud_py_pb2",
    api_version = 2,
    deps = [":multichannel_cloud_proto"],
)

cc_proto_library(
    name = "multichannel_cloud_cc_proto",
    deps = [":multichannel_cloud_proto"],
)

blueproto_library(
    name = "multichannel_cloud_blueproto",
    deps = [
        ":multichannel_cloud_proto",
        "//googlex/proxy/blueproto:blueproto_eigen_support",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_blueproto",
        "//googlex/proxy/module_system/core:common_blueproto",
    ],
)

pyclif_proto_library(
    name = "multichannel_cloud_pyclif",
    proto_lib = ":multichannel_cloud_proto",
    proto_srcfile = "multichannel_cloud.proto",
)

proto_library(
    name = "detected_objects_proto",
    srcs = ["detected_objects.proto"],
    cc_api_version = 2,
    visibility = INTERFACE_VISIBILITY,
    deps = [
        "//googlex/proxy/blueproto:eigen_proto",
        "//googlex/proxy/eigenmath/proto:plane3d_proto",
        "//googlex/proxy/module_system/core:common_proto",
    ],
)

cc_proto_library(
    name = "detected_objects_cc_proto",
    deps = [":detected_objects_proto"],
)

proto_library(
    name = "plane_estimator_proto",
    srcs = [
        "proto/plane_estimator.proto",
    ],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:gce"],
)

cc_proto_library(
    name = "plane_estimator_cc_proto",
    deps = [":plane_estimator_proto"],
)

blueproto_library(
    name = "plane_estimator_blueproto",
    deps = [
        ":plane_estimator_proto",
    ],
)

proto_library(
    name = "ego_detector_config_proto",
    srcs = ["ego_detector_config.proto"],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:vendor"],
    deps = ["//googlex/proxy/module_system/robot_config:robot_proto"],
)

cc_proto_library(
    name = "ego_detector_config_cc_proto",
    deps = [":ego_detector_config_proto"],
)

proto_library(
    name = "camera_point_cloud_segmentation_config_proto",
    srcs = ["camera_point_cloud_segmentation_config.proto"],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:vendor"],
    deps = [
        ":plane_classification_config_proto",
        ":region_segmentation_config_proto",
        "//googlex/proxy/module_system/robot_config:robot_proto",
    ],
)

cc_proto_library(
    name = "camera_point_cloud_segmentation_config_cc_proto",
    deps = [":camera_point_cloud_segmentation_config_proto"],
)

proto_library(
    name = "point_cloud_segmentation_config_proto",
    srcs = ["point_cloud_segmentation_config.proto"],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:vendor"],
    deps = [
        ":plane_classification_config_proto",
        ":region_segmentation_config_proto",
        "//googlex/proxy/module_system/robot_config:robot_proto",
    ],
)

cc_proto_library(
    name = "point_cloud_segmentation_config_cc_proto",
    deps = [":point_cloud_segmentation_config_proto"],
)

proto_library(
    name = "plane_classification_config_proto",
    srcs = ["plane_classification_config.proto"],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:vendor"],
    deps = ["//googlex/proxy/module_system/robot_config:robot_proto"],
)

cc_proto_library(
    name = "plane_classification_config_cc_proto",
    deps = [":plane_classification_config_proto"],
)

proto_library(
    name = "region_segmentation_config_proto",
    srcs = ["region_segmentation_config.proto"],
    cc_api_version = 2,
    compatible_with = ["//buildenv/target:vendor"],
    deps = ["//googlex/proxy/module_system/robot_config:robot_proto"],
)

cc_proto_library(
    name = "region_segmentation_config_cc_proto",
    deps = [":region_segmentation_config_proto"],
)

cc_library(
    name = "point_cloud_visualization",
    srcs = [
        "visualization_helpers.cc",
    ],
    hdrs = [
        "visualization_helpers.h",
    ],
    linkstatic = 0,
    deps = [
        ":algorithms",
        ":cluster_region",
        ":indices",
        ":multichannel_cloud",
        ":planar_region",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/textlog:logger",
        "//googlex/proxy/visualization:marker_utils",
        "//googlex/proxy/visualization:visualization_client",
        "//image/content/color/hsv:hsv-lib",
        "//third_party/absl/strings",
    ],
)

robot_module(
    name = "point_cloud_segmentation_module",
    srcs = [
        "point_cloud_segmentation_module.cc",
    ],
    cls = "blue::mobility::PointCloudSegmentationModule",
    interface = "point_cloud_segmentation_module.h",
    deps = [
        ":algorithms",
        ":camera_conversions",
        ":cloud",
        ":cluster_region",
        ":detected_objects",
        ":multichannel_cloud",
        ":multichannel_cloud_cc_proto",
        ":planar_region",
        ":plane_classification",
        ":plane_classification_config_cc_proto",
        ":point_cloud_segmentation_config_cc_proto",
        ":point_cloud_segmentation_module_utils",
        ":point_cloud_visualization",
        ":segmentation",
        "//googlex/proxy/control/proto:safety_detector_blueproto",
        "//googlex/proxy/control/proto:safety_detector_cc_proto",
        "//googlex/proxy/ego_masking:ego_mask_cc_proto",
        "//googlex/proxy/module_system/core:module",
        "//googlex/proxy/module_system/message_sync",
        "//googlex/proxy/navigation/differential_drive:base_kinematics_config_cc_proto",
        "//googlex/proxy/navigation/differential_drive:diff_drive_cc_proto",
        "//googlex/proxy/object_properties/visual_vocabularies:semantic_types",
        "//googlex/proxy/robot_config:robot",
        "//googlex/proxy/textlog:logger",
        "//googlex/proxy/transform_tree",
        "//googlex/proxy/transform_tree:transform_frames",
        "//googlex/proxy/transform_tree:transform_listener",
        "//googlex/proxy/visualization:visualization_client",
        "//third_party/absl/functional:bind_front",
        "//third_party/absl/types:span",
    ],
)

cc_test(
    name = "point_cloud_segmentation_module_test",
    srcs = ["point_cloud_segmentation_module_test.cc"],
    deps = [
        ":detected_objects_cc_proto",
        ":point_cloud_segmentation_module_library",
        "//googlex/proxy/conversions",
        "//googlex/proxy/ego_masking:ego_mask_cc_proto",
        "//googlex/proxy/environment/test_utils:blue_environment_test",
        "//googlex/proxy/environment/test_utils:blue_test_main",
        "//googlex/proxy/module_system/core:module",
        "//googlex/proxy/module_system/core:tests_helper_module",
        "//googlex/proxy/robot_config:get_test_robot_config",
        "//googlex/proxy/visualization:visualization_client",
        "//net/proto2/contrib/parse_proto:parse_text_proto",
        "//testing/base/public:gunit",
    ],
)

cc_library(
    name = "camera_conversions",
    srcs = ["camera_conversions.cc"],
    hdrs = ["camera_conversions.h"],
    deps = [
        ":algorithms",
        ":multichannel_cloud",
        ":segmentation",
        "//base",
        "//googlex/proxy/conversions:perception_conversions",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_blueproto",
        "//googlex/proxy/image_utils:image",
        "//googlex/proxy/low_level_perception/camera_model",
        "//googlex/proxy/low_level_perception/camera_model:camera_model_conversions",
        "//googlex/proxy/low_level_perception/proto:config_blueproto",
        "//googlex/proxy/low_level_perception/proto:vision_blueproto",
    ],
)

cc_test(
    name = "camera_conversions_test",
    size = "small",
    srcs = ["camera_conversions_test.cc"],
    deps = [
        ":camera_conversions",
        "//googlex/proxy/image_utils:image",
        "//googlex/proxy/low_level_perception/proto:config_blueproto",
        "//googlex/proxy/low_level_perception/proto:vision_blueproto",
        "//testing/base/public:gunit_main",
    ],
)

cc_library(
    name = "point_cloud_segmentation_module_utils",
    srcs = [
        "point_cloud_segmentation_module_utils.cc",
    ],
    hdrs = [
        "point_cloud_segmentation_module_utils.h",
    ],
    deps = [
        ":cluster_region",
        ":detected_objects",
        ":detected_objects_cc_proto",
        ":mean_shift_segmentation",
        ":multichannel_cloud",
        ":plane_classification",
        ":point_cloud_segmentation_config_cc_proto",
        "//googlex/proxy/ego_masking:ego_detector",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/object_properties/visual_vocabularies:semantic_types",
        "//third_party/absl/strings",
        "//third_party/absl/types:optional",
        "//third_party/absl/types:span",
    ],
)

cc_test(
    name = "point_cloud_segmentation_module_utils_test",
    srcs = ["point_cloud_segmentation_module_utils_test.cc"],
    deps = [
        ":point_cloud_segmentation_module_utils",
        ":segmentation",
        "//testing/base/public:gunit_main",
    ],
)

robot_module(
    name = "camera_point_cloud_segmentation_module",
    srcs = ["camera_point_cloud_segmentation_module.cc"],
    cls = "blue::mobility::CameraPointCloudSegmentationModule",
    interface = "camera_point_cloud_segmentation_module.h",
    deps = [
        ":camera_conversions",
        ":camera_point_cloud_segmentation_config_cc_proto",
        ":cluster_region",
        ":detected_objects",
        ":multichannel_cloud",
        ":planar_region",
        ":plane_classification_config_cc_proto",
        ":point_cloud_segmentation_module_utils",
        ":point_cloud_visualization",
        "//googlex/proxy/clock:averaging_timer",
        "//googlex/proxy/control/proto:safety_detector_blueproto",
        "//googlex/proxy/control/proto:safety_detector_cc_proto",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_blueproto",
        "//googlex/proxy/ego_masking:ego_mask_cc_proto",
        "//googlex/proxy/ego_masking:ego_masking_message_sync",
        "//googlex/proxy/eigenmath/proto:twist_blueproto",
        "//googlex/proxy/eigenmath/proto:twist_cc_proto",
        "//googlex/proxy/eigenmath/proto:wrench_blueproto",
        "//googlex/proxy/eigenmath/proto:wrench_cc_proto",
        "//googlex/proxy/low_level_perception/proto:vision_blueproto",
        "//googlex/proxy/low_level_perception/proto:vision_cc_proto",
        "//googlex/proxy/low_level_perception/stereo_server:stereo_client",
        "//googlex/proxy/module_system/core:module",
        "//googlex/proxy/module_system/robot_config:robot_cc_proto",
        "//googlex/proxy/navigation/differential_drive:base_kinematics_config_cc_proto",
        "//googlex/proxy/navigation/differential_drive:diff_drive_cc_proto",
        "//googlex/proxy/object_properties/visual_vocabularies:semantic_types",
        "//googlex/proxy/primitives:l3_robot_state_blueproto",
        "//googlex/proxy/robot_config:robot",
        "//googlex/proxy/transform_tree",
        "//googlex/proxy/transform_tree:transform_frames",
        "//googlex/proxy/transform_tree:transform_listener",
        "//googlex/proxy/visualization:visualization_client",
        "//third_party/absl/types:optional",
        "//third_party/blue/machine/hilo/proto:stereo_and_robot_state_cc_proto",
    ],
)

cc_test(
    name = "camera_point_cloud_segmentation_module_test",
    srcs = ["camera_point_cloud_segmentation_module_test.cc"],
    deps = [
        ":camera_point_cloud_segmentation_config_cc_proto",
        ":camera_point_cloud_segmentation_module_library",
        ":detected_objects",
        "//googlex/proxy/conversions",
        "//googlex/proxy/conversions/legacy_proto:geometry_types_proto",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_blueproto",
        "//googlex/proxy/environment/test_utils:blue_environment_test",
        "//googlex/proxy/environment/test_utils:blue_test_main",
        "//googlex/proxy/low_level_perception/proto:config_blueproto",
        "//googlex/proxy/low_level_perception/proto:vision_blueproto",
        "//googlex/proxy/low_level_perception/stereo_server",
        "//googlex/proxy/module_system/core:module",
        "//googlex/proxy/module_system/core:tests_helper_module",
        "//googlex/proxy/primitives:l3_robot_state_blueproto",
        "//googlex/proxy/primitives:l3_robot_state_cc_proto",
        "//googlex/proxy/primitives/client:l3_test_utils",
        "//googlex/proxy/robot_config:get_test_robot_config",
        "//googlex/proxy/transform_tree:transform_frames",
        "//testing/base/public:gunit",
    ],
)

cc_library(
    name = "planar_region",
    srcs = ["planar_region.cc"],
    hdrs = ["planar_region.h"],
    deps = [
        ":algorithms",
        ":cloud",
        ":plane_estimator",
        ":region_segmentation_config_cc_proto",
        ":segmentation",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/object_properties/visual_vocabularies:semantic_types",
        "//third_party/absl/container:flat_hash_set",
        "//util/math:mathutil",
    ],
)

cc_library(
    name = "plane_classification",
    srcs = ["plane_classification.cc"],
    hdrs = ["plane_classification.h"],
    deps = [
        ":planar_region",
        ":plane_classification_config_cc_proto",
        "//googlex/proxy/eigenmath",
        "//util/math:mathutil",
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
        "//base",
        "//googlex/proxy/eigenmath",
        "//third_party/eigen3",
    ],
)

cc_test(
    name = "plane_estimator_test",
    srcs = ["plane_estimator_test.cc"],
    deps = [
        ":multichannel_cloud",
        ":plane_estimator",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/eigenmath:matchers",
        "//testing/base/public:gunit_main",
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
        "//googlex/proxy/clock:time",
        "//googlex/proxy/conversions",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/module_system/core:common_cc_proto",
    ],
)

cc_test(
    name = "multichannel_cloud_test",
    size = "small",
    srcs = ["multichannel_cloud_test.cc"],
    deps = [
        ":multichannel_cloud",
        "//googlex/proxy/blueproto:eigen_cc_proto",
        "//googlex/proxy/conversions/legacy_proto:legacy_proto_cc_proto",
        "//googlex/proxy/eigenmath:matchers",
        "//net/proto2/contrib/parse_proto:parse_text_proto",
        "//testing/base/public:gunit_main",
    ],
)

cc_library(
    name = "cloud",
    hdrs = ["cloud.h"],
    deps = [
        "//base",
        "//googlex/proxy/eigenmath",
        "//third_party/eigen3",
    ],
)

cc_test(
    name = "cloud_test",
    size = "small",
    srcs = ["cloud_test.cc"],
    deps = [
        ":cloud",
        "//googlex/proxy/eigenmath:matchers",
        "//testing/base/public:gunit_main",
    ],
)

cc_library(
    name = "cloud_proto_utils",
    hdrs = ["cloud_proto_utils.h"],
    deps = [
        ":cloud",
        "//net/proto2/public:proto2",
    ],
)

cc_library(
    name = "algorithms",
    hdrs = ["algorithms.h"],
    deps = [
        ":indices",
        ":plane_estimator",
        "//googlex/proxy/containers",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/textlog:logger",
        "//third_party/eigen3",
        "//util/geometry2d:convex-hull",
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
        "//base",
    ],
)

cc_library(
    name = "detected_objects",
    hdrs = ["detected_objects.h"],
    deps = [
        ":cloud",
        ":cloud_proto_utils",
        ":detected_objects_cc_proto",
        ":planar_region",
        "//googlex/proxy/blueproto:eigen_cc_proto",
        "//googlex/proxy/conversions",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/eigenmath/proto:plane3d_cc_proto",
        "//googlex/proxy/textlog:logger",
        "//googlex/proxy/transform_tree:transform_frames",
        "//third_party/absl/strings",
        "//util/task:status",
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
        "//base",
        "//googlex/proxy/eigenmath",
    ],
)

cc_library(
    name = "mean_shift_segmentation",
    srcs = ["mean_shift_segmentation.cc"],
    hdrs = ["mean_shift_segmentation.h"],
    deps = [
        ":cluster_region",
        ":region_segmentation_config_cc_proto",
        "//googlex/proxy/eigenmath",
        "//third_party/eigen3",
    ],
)

cc_library(
    name = "image_conversions",
    srcs = ["image_conversions.cc"],
    hdrs = ["image_conversions.h"],
    deps = [
        ":multichannel_cloud",
        "//googlex/proxy/image_utils:image",
        "//third_party/absl/status:statusor",
    ],
)

cc_test(
    name = "image_conversions_test",
    srcs = ["image_conversions_test.cc"],
    deps = [
        ":image_conversions",
        "//testing/base/public:gunit_main",
        "//third_party/absl/status:statusor",
    ],
)

filegroup(
    name = "testdata",
    testonly = 1,
    data = ["testdata/cbr_multichannel_cloud_proto.pb"],
)

cc_test(
    name = "algorithms_test",
    srcs = ["algorithms_test.cc"],
    data = [":testdata"],
    deps = [
        ":algorithms",
        ":cloud",
        ":multichannel_cloud",
        ":multichannel_cloud_cc_proto",
        ":plane_estimator",
        "//devtools/build/runtime:get_runfiles_dir",
        "//file/base",
        "//googlex/proxy/eigenmath",
        "//googlex/proxy/eigenmath:matchers",
        "//testing/base/public:gunit_main",
        "//util/math:mathutil",
    ],
)

proto_library(
    name = "point_segmentations_proto",
    srcs = [
        "point_segmentations.proto",
    ],
    cc_api_version = 2,
    deps = [
        "//googlex/proxy/blueproto:eigen_proto",
        "//googlex/proxy/module_system/core:common_proto",
        "//third_party/robotics/logging:proto_options",
    ],
)

cc_proto_library(
    name = "point_segmentations_cc_proto",
    deps = [":point_segmentations_proto"],
)

go_proto_library(
    name = "point_segmentations_go_proto",
    deps = [":point_segmentations_proto"],
)

py_proto_library(
    name = "point_segmentations_py_pb2",
    api_version = 2,
    deps = [":point_segmentations_proto"],
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
        "//testing/base/public:gunit_main",
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
        "//testing/base/public:gunit",
    ],
)
