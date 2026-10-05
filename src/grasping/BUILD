load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "types",
    srcs = [
        "__init__.py",
        "types.py",
    ],
)

py_library(
    name = "contact_pairs",
    srcs = ["contact_pairs.py"],
    deps = [":types"],
)
