load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:container.bzl", "container_image")
load("@intrinsic-core//bazel:python.bzl", "py_library")
load("@intrinsic-core//intrinsic_sdk/intrinsic/skills/build_defs:skill.bzl", "py_skill", "skill_manifest")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "control_suction_gripper_proto",
    srcs = ["control_suction_gripper.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/assets/proto:field_metadata_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:resolved_dependency_proto",
        "@intrinsic_apis//intrinsic/hardware/gripper/eoat:eoat_service_proto",
    ],
)

py_proto_library(
    name = "control_suction_gripper_py_pb2",
    deps = [":control_suction_gripper_proto"],
)

skill_manifest(
    name = "control_suction_gripper_manifest",
    src = "control_suction_gripper_manifest.textproto",
    deps = [":control_suction_gripper_proto"],
)

py_library(
    name = "control_suction_gripper",
    srcs = ["control_suction_gripper.py"],
    deps = [
        ":control_suction_gripper_py_pb2",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets:interface_utils_py",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets/dependencies:utils_py",
        "@intrinsic-core//intrinsic_sdk/intrinsic/skills/python:skill_interface",
        "@intrinsic_apis//intrinsic/hardware/gripper/eoat:eoat_service_py_pb2",
        "@intrinsic_apis//intrinsic/hardware/gripper/eoat:eoat_service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/skills/proto:footprint_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

container_image(
    name = "python3_base_image",
    base = "@distroless_python3",
)

py_skill(
    name = "control_suction_gripper_skill",
    base_image = ":python3_base_image",
    manifest = ":control_suction_gripper_manifest",
    deps = [
        ":control_suction_gripper",
        ":control_suction_gripper_py_pb2",
    ],
)
