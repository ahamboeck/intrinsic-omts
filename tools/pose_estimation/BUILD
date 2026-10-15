load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "cycle_overlay",
    srcs = ["cycle_overlay.py"],
    deps = [
        ":pose_snapshot",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:proto_building",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/proto:object_world_service_py_pb2",
    ],
)

py_library(
    name = "viewpoint_benchmark",
    srcs = ["viewpoint_benchmark.py"],
    deps = ["//src/grasping:geometry"],
)

py_library(
    name = "viewpoint_images",
    srcs = ["viewpoint_images.py"],
    deps = [
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@ai_intrinsic_sdks_pip_deps//pillow",
    ],
)

py_library(
    name = "run_viewpoint_benchmark_lib",
    srcs = ["run_viewpoint_benchmark.py"],
    deps = [
        ":viewpoint_benchmark",
        ":viewpoint_images",
        "//src/core:config",
        "//src/grasping:types",
        "//src/hardware:robot",
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@ai_intrinsic_sdks_pip_deps//pillow",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/client/v1/python/camera:cameras",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/python:pose_estimation_client_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets:id_utils_py",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:rotation3",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:errors",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//intrinsic_sdk/intrinsic/util/grpc:interceptor",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/proto:object_world_service_py_pb2_grpc",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/python:object_world_client",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimation_service_py_pb2_grpc",
    ],
)

py_binary(
    name = "run_viewpoint_benchmark",
    srcs = ["run_viewpoint_benchmark.py"],
    main = "run_viewpoint_benchmark.py",
    deps = [":run_viewpoint_benchmark_lib"],
)

py_library(
    name = "pose_snapshot",
    srcs = ["pose_snapshot.py"],
    deps = [
        "//src/grasping:geometry",
        "//tools/grasping:pose_overlay_lib",
    ],
)

py_library(
    name = "run_pose_estimation_lib",
    srcs = ["run_pose_estimation.py"],
    deps = [
        ":pose_snapshot",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets:id_utils_py",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/proto:object_world_service_py_pb2_grpc",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/python:object_world_client",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimator_id_py_pb2",
    ],
)

py_binary(
    name = "run_pose_estimation",
    srcs = ["run_pose_estimation.py"],
    main = "run_pose_estimation.py",
    deps = [
        ":run_pose_estimation_lib",
    ],
)

py_library(
    name = "register_using_train_service_lib",
    srcs = ["register_using_train_service.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_googleapis//google/longrunning:operations_py_proto",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service:ioc_train_service_lib",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/proto:ioc_pose_estimator_params_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets:id_utils_py",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets/proto:installed_assets_pb2_grpc",
        "@intrinsic-core//intrinsic_sdk/intrinsic/assets/proto:installed_assets_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:data_asset_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:id_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:view_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimation_config_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_range_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:target_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:train_service_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:train_service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_py_pb2",
    ],
)

py_binary(
    name = "register_using_train_service",
    srcs = ["register_using_train_service.py"],
    main = "register_using_train_service.py",
    deps = [
        ":register_using_train_service_lib",
    ],
)
