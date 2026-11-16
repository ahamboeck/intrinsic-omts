load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:container.bzl", "container_image")
load("@intrinsic-core//bazel:python.bzl", "py_library")
load("@intrinsic-core//intrinsic_sdk/intrinsic/skills/build_defs:skill.bzl", "py_skill", "skill_manifest")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "suction_mesh_plan_grasp_proto",
    srcs = ["suction_mesh_plan_grasp.proto"],
    deps = [
        "//src/grasping:gripper_model_proto",
        "@intrinsic_apis//intrinsic/assets/proto:field_metadata_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:resolved_dependency_proto",
    ],
)

py_proto_library(
    name = "suction_mesh_plan_grasp_py_pb2",
    deps = [":suction_mesh_plan_grasp_proto"],
)

skill_manifest(
    name = "suction_mesh_plan_grasp_manifest",
    src = "suction_mesh_plan_grasp_manifest.textproto",
    deps = [":suction_mesh_plan_grasp_proto"],
)

py_library(
    name = "suction_mesh_plan_grasp",
    srcs = ["suction_mesh_plan_grasp.py"],
    deps = [
        ":suction_mesh_plan_grasp_py_pb2",
        "//src/grasping:feasibility",
        "//src/grasping:frames",
        "//src/grasping:workpiece_mesh",
        "//src/grasping/planners/suction_mesh:candidates",
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
    name = "suction_mesh_plan_grasp_skill",
    base_image = ":python3_base_image",
    manifest = ":suction_mesh_plan_grasp_manifest",
    deps = [
        ":suction_mesh_plan_grasp",
        ":suction_mesh_plan_grasp_py_pb2",
    ],
)
