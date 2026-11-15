load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "gripper_model_proto",
    srcs = ["gripper_model.proto"],
)

py_proto_library(
    name = "gripper_model_py_pb2",
    deps = [":gripper_model_proto"],
)

py_library(
    name = "interface",
    srcs = [
        "__init__.py",
        "interface.py",
    ],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "registry",
    srcs = ["registry.py"],
    deps = [
        ":interface",
        "//src/core:config",
        "//src/grasping/planners/antipodal_mesh:adapter",
        "//src/grasping/planners/cuboid_center:adapter",
        "//src/grasping/planners/moveit:adapter",
    ],
)

py_library(
    name = "workpiece_mesh",
    srcs = ["workpiece_mesh.py"],
    deps = [
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@intrinsic-core//intrinsic_sdk/intrinsic/geometry/proto:geometry_service_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
    ],
)

py_library(
    name = "feasibility",
    srcs = ["feasibility.py"],
    deps = [
        "//src/utils:math_utils",
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/proto/v1:motion_specification_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/motion_planning:motion_planner_client",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/motion_planning/proto:motion_target_py_pb2",
        "@intrinsic_apis//intrinsic/motion_planning/proto/v1:robot_specification_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2",
    ],
)
