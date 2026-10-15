load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

# Pure marker geometry is tested in Bazel; the CLI runs in a ROS environment.
py_library(
    name = "pose_overlay_lib",
    srcs = ["pose_overlay.py"],
    deps = [
        "//src/grasping:geometry",
        "//src/grasping:types",
    ],
)

py_library(
    name = "mesh_demo_lib",
    srcs = ["mesh_demo.py"],
    deps = [
        "//src/grasping:contact_pairs",
        "//src/grasping:geometry",
        "//src/grasping:hande_profile",
        "//src/grasping:local_checks",
        "//src/grasping:poses",
        "//src/grasping:types",
    ],
)

py_binary(
    name = "mesh_demo",
    srcs = ["mesh_demo.py"],
    main = "mesh_demo.py",
    deps = [":mesh_demo_lib"],
)
