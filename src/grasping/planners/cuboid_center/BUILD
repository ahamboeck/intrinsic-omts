load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "algorithm",
    srcs = ["algorithm.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "adapter",
    srcs = ["adapter.py"],
    deps = [
        ":algorithm",
        "//src/core:config",
        "//src/grasping:interface",
        "//src/utils:script_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)
