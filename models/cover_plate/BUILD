load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "cover_plate_filegroup",
    srcs = ["cover_plate.glb"],
)

sdf_scene_object(
    name = "cover_plate_scene_object",
    src = "cover_plate.sdf",
    sdf_assets = [":cover_plate_filegroup"],
)

intrinsic_scene_object(
    name = "cover_plate",
    manifest = ":cover_plate_manifest.textproto",
    scene_object = ":cover_plate_scene_object",
)
