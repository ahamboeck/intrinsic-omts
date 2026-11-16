load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "candidates",
    srcs = ["candidates.py"],
    deps = [
        "//src/grasping:workpiece_mesh",
        "@ai_intrinsic_sdks_pip_deps//numpy",
    ],
)

py_library(
    name = "adapter",
    srcs = ["adapter.py"],
    deps = [
        "//src/core:config",
        "//src/grasping:gripper_model_py_pb2",
        "//src/grasping:interface",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)
