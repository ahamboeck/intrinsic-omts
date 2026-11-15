load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "math_utils",
    srcs = ["math_utils.py"],
    deps = [
        "//src/core:types",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "script_utils",
    srcs = ["script_utils.py"],
    deps = [
        ":world_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:proto_building",
    ],
)

py_library(
    name = "world_utils",
    srcs = ["world_utils.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)

py_library(
    name = "workpiece_pose",
    srcs = ["workpiece_pose.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "execution_utils",
    srcs = ["execution_utils.py"],
    deps = [
        "//src/core:types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_library(
    name = "random_placement",
    srcs = ["random_placement.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "regrasp_frames",
    srcs = ["regrasp_frames.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "utils",
    srcs = ["__init__.py"],
    deps = [
        ":execution_utils",
        ":math_utils",
        ":random_placement",
        ":regrasp_frames",
        ":script_utils",
        ":workpiece_pose",
        ":world_utils",
    ],
)
