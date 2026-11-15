load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "adapter",
    srcs = ["adapter.py"],
    deps = [
        "//src/core:config",
        "//third_party/intrinsic_moveit:moveit_grasp_planner",
    ],
)
