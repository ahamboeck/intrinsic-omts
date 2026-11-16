load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:container.bzl", "container_image")
load("@intrinsic-core//bazel:python.bzl", "py_library")
load("@intrinsic-core//intrinsic_sdk/intrinsic/skills/build_defs:skill.bzl", "py_skill", "skill_manifest")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "antipodal_mesh_plan_grasp_proto",
    srcs = ["antipodal_mesh_plan_grasp.proto"],
    deps = [
        "//src/grasping:gripper_model_proto",
        "@intrinsic_apis//intrinsic/assets/proto:field_metadata_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:resolved_dependency_proto",
    ],
)

py_proto_library(
    name = "antipodal_mesh_plan_grasp_py_pb2",
    deps = [":antipodal_mesh_plan_grasp_proto"],
)

skill_manifest(
    name = "antipodal_mesh_plan_grasp_manifest",
    src = "antipodal_mesh_plan_grasp_manifest.textproto",
    deps = [":antipodal_mesh_plan_grasp_proto"],
)

py_library(
    name = "antipodal_mesh_plan_grasp",
    srcs = ["antipodal_mesh_plan_grasp.py"],
    deps = [
        ":antipodal_mesh_plan_grasp_py_pb2",
        "//src/grasping:feasibility",
        "//src/grasping:frames",
        "//src/grasping:workpiece_mesh",
        "//src/grasping/planners/antipodal_mesh:candidates",
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic_sdk/intrinsic/motion_planning:motion_planner_client",
        "@intrinsic-core//intrinsic_sdk/intrinsic/skills/python:skill_interface",
        "@intrinsic_apis//intrinsic/skills/proto:footprint_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2",
    ],
)

container_image(
    name = "python3_base_image",
    base = "@distroless_python3",
)

py_skill(
    name = "antipodal_mesh_plan_grasp_skill",
    base_image = ":python3_base_image",
    manifest = ":antipodal_mesh_plan_grasp_manifest",
    deps = [
        ":antipodal_mesh_plan_grasp",
        ":antipodal_mesh_plan_grasp_py_pb2",
    ],
)
