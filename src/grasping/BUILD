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

py_library(
    name = "geometry",
    srcs = ["geometry.py"],
    deps = [":types"],
)

py_library(
    name = "profile",
    srcs = ["profile.py"],
    deps = [
        ":geometry",
        ":types",
    ],
)

py_library(
    name = "hande_profile",
    srcs = ["hande_profile.py"],
    deps = [
        ":geometry",
        ":profile",
        ":types",
    ],
)

py_library(
    name = "poses",
    srcs = ["poses.py"],
    deps = [
        ":geometry",
        ":profile",
        ":types",
    ],
)

py_library(
    name = "local_checks",
    srcs = ["local_checks.py"],
    deps = [
        ":contact_pairs",
        ":geometry",
        ":poses",
        ":profile",
        ":types",
    ],
)
