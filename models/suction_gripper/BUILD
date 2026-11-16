load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")

package(default_visibility = ["//visibility:public"])

exports_files(["suction_gripper.sdf"])

sdf_scene_object(
    name = "suction_gripper_sdf_scene_object",
    src = "suction_gripper.sdf",
)

intrinsic_scene_object(
    name = "suction_gripper_scene_object",
    manifest = "suction_gripper_scene_object_manifest.textproto",
    scene_object = ":suction_gripper_sdf_scene_object",
)

# Core's EOAT gripper service. In simulation it drives Gazebo's suction
# plugin over the simulator's GPIO service; on hardware it would drive the
# signals through the realtime control service named in the config.
intrinsic_service(
    name = "suction_gripper_service",
    default_config = "suction_gripper_service_default_config.pbtxt",
    images = [
        "@intrinsic-core//intrinsic_hardware/intrinsic/hardware/gripper/service_asset:gripper-service-image.tar",
    ],
    manifest = "suction_gripper_service_manifest.textproto",
    deps = [
        "@intrinsic-core//intrinsic_hardware/intrinsic/hardware/gripper/service_asset:suction_gripper_realtime_control_service_config_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_state_proto",
        "@intrinsic_apis//intrinsic/hardware/gpio/v1:gpio_service_proto",
        "@intrinsic_apis//intrinsic/hardware/gripper/eoat:eoat_service_proto",
    ],
)

intrinsic_hardware_device(
    name = "suction_gripper",
    assets = [
        ":suction_gripper_scene_object",
        ":suction_gripper_service",
    ],
    manifest = "suction_gripper_manifest.textproto",
)
