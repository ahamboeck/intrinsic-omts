"""Opt-in BB-01 simulation camera orbit; never picks or changes object belief.

The same host collector can target stock and modified deployed services. A saved
plan defines acquisition only; simulator truth is never an inference pose prior.
"""

import argparse
import csv
import hashlib
import json
import math
import time
from datetime import UTC, datetime, timedelta
from pathlib import Path

from tools.pose_estimation.viewpoint_benchmark import (
  make_plan,
  pair_results,
  plan_digest,
  pose_errors,
  record,
  transform,
)


def write_json(path, value):
  path.write_text(json.dumps(value, indent=2, allow_nan=False))


def sdk_transform(pose):
  return transform(
    dict(
      translation_m=[float(x) for x in pose.translation],
      rotation_basis_columns=[
        [float(x) for x in pose.rotation.rotate_point(axis)]
        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))
      ],
    )
  )


def flatten_pose(prefix, pose):
  if pose is None:
    return {}
  return {
    **{
      f"{prefix}_{a}_m": v for a, v in zip("xyz", pose.translation, strict=True)
    },
    **{
      f"{prefix}_r{row}{col}": pose.rotation[col][row]
      for row in range(3)
      for col in range(3)
    },
  }


def require_idle_simulation(solution):
  if solution.is_simulated is not True:
    raise RuntimeError("Refusing benchmark on a non-simulated deployment.")
  if solution.executive.has_operation and not solution.executive.operation.done:
    raise RuntimeError(
      "Executive busy; refusing to replace an active operation."
    )


def connect(args):
  from intrinsic.solutions import deployments
  from intrinsic.world.proto import object_world_service_pb2_grpc
  from intrinsic.world.python.object_world_client import ObjectWorldClient

  solution = deployments.connect(address=args.address)
  require_idle_simulation(solution)
  simulator = ObjectWorldClient(
    "sim_world",
    object_world_service_pb2_grpc.ObjectWorldServiceStub(solution.grpc_channel),
  )
  return solution, simulator


def actual_pose(simulator, name):
  return sdk_transform(
    simulator.get_transform(simulator.root, simulator.get_object(name))
  )


def camera_client(solution, camera_name):
  import grpc
  from intrinsic.perception.client.v1.python.camera import cameras
  from intrinsic.util.grpc import interceptor

  handle = solution.resources[camera_name].proto
  # Route through the connected deployment, preserving service-specific headers.
  from intrinsic.perception.service.python import pose_estimation_client_utils

  params = pose_estimation_client_utils.get_connection_params(
    handle.connection_info
  )
  channel = grpc.intercept_channel(
    solution.grpc_channel, interceptor.HeaderAdderInterceptor(params.headers)
  )
  return cameras.Camera.create_from_resource_handle(
    handle, solution.world, channel=channel
  )


def create_plan(args):
  solution, simulator = connect(args)
  camera = camera_client(solution, args.camera)
  # Select intensity sensor explicitly, not the driver's sensor ordering.
  optical = camera.world_t_sensor(
    camera.sensor_id_to_name[int(args.rgb_sensor)]
  )
  if optical is None:
    raise ValueError("Camera has no world optical transform.")
  actual = actual_pose(simulator, args.object)
  plan = make_plan(
    actual, sdk_transform(optical), args.views, args.radius, args.height
  )
  plan.update(
    camera_name=args.camera,
    object_name=args.object,
    rgb_sensor=args.rgb_sensor,
    depth_sensor=args.depth_sensor,
    tool_object=args.tool_object,
    tool_frame=args.tool_frame,
  )
  # Saved hand-eye transform maps optical sensor coordinates into tool frame.
  world_tool = sdk_transform(
    solution.world.get_transform(
      solution.world.root,
      solution.world.get_frame(args.tool_frame, args.tool_object),
    )
  )
  plan["tool_t_camera"] = record(
    world_tool.inverse().compose(sdk_transform(optical))
  )
  args.plan.parent.mkdir(parents=True, exist_ok=True)
  with args.plan.open("x") as stream:
    json.dump(plan, stream, indent=2, allow_nan=False)
  print(
    f"Saved {len(plan['views'])} targets to {args.plan}. No motion executed."
  )


def collect(args):
  if not args.execute_simulation:
    raise ValueError(
      "run requires --execute-simulation; this moves the simulated arm."
    )
  import grpc
  import numpy as np
  from google.protobuf import json_format
  from intrinsic.assets import id_utils
  from intrinsic.math.python import proto_conversion, rotation3
  from intrinsic.perception.proto.v1 import pose_estimation_service_pb2_grpc
  from intrinsic.perception.service.python import pose_estimation_client_utils
  from intrinsic.solutions import errors as solutions_errors
  from intrinsic.util.grpc import interceptor
  from PIL import Image

  from src.core.config import RobotConfig
  from src.grasping.types import TriangleMesh
  from src.hardware.robot import Robot
  from tools.pose_estimation.viewpoint_images import (
    cad_in_image,
    comparison_image,
  )

  plan = json.loads(args.plan.read_text())
  cad = json.loads(args.cad.read_text())
  TriangleMesh(
    tuple(map(tuple, cad["vertices"])), tuple(map(tuple, cad["triangles"]))
  )
  provenance = json.loads(args.provenance.read_text())
  if not provenance:
    raise ValueError(
      "Record deployed source/artifact identities in --provenance."
    )
  # Reject malformed targets before any movement.
  hand_eye = transform(plan["tool_t_camera"])
  center = transform(plan["center_pose"])
  targets = [
    transform(v["camera_pose"]).compose(hand_eye.inverse())
    for v in plan["views"]
  ]
  if not targets:
    raise ValueError("Plan has no views.")
  solution, simulator = connect(args)
  camera = camera_client(solution, plan["camera_name"])
  rgb_name = camera.sensor_id_to_name[int(plan["rgb_sensor"])]
  depth_name = camera.sensor_id_to_name[int(plan["depth_sensor"])]
  world_tool = sdk_transform(
    solution.world.get_transform(
      solution.world.root,
      solution.world.get_frame(plan["tool_frame"], plan["tool_object"]),
    )
  )
  optical_now = sdk_transform(camera.world_t_sensor(rgb_name))
  calibration_error = pose_errors(
    world_tool.inverse().compose(optical_now), hand_eye
  )
  if (
    calibration_error["position_error_mm"] > 0.1
    or calibration_error["rotation_error_deg"] > 0.1
  ):
    raise ValueError(
      "Camera/tool calibration differs from saved plan; do not compare different setups."
    )
  handle = solution.resources[args.service].proto
  params = pose_estimation_client_utils.get_connection_params(
    handle.connection_info
  )
  channel = grpc.intercept_channel(
    solution.grpc_channel, interceptor.HeaderAdderInterceptor(params.headers)
  )
  stub = pose_estimation_service_pb2_grpc.PoseEstimationServiceStub(channel)
  robot = Robot(
    solution,
    RobotConfig(
      arm_part_name=args.arm_part,
      tool_object_name=plan["tool_object"],
      tool_frame_name=plan["tool_frame"],
    ),
  )
  args.output.mkdir(parents=True, exist_ok=False)
  plan_id = plan_digest(plan)
  metadata = dict(
    label=args.label,
    plan_id=plan_id,
    address=args.address,
    started_at_utc=datetime.now(UTC).isoformat(),
    estimate_frame=args.estimate_frame,
    estimator_id=args.estimator,
    provenance=provenance,
    provenance_verified=False,
    cad_sha256=hashlib.sha256(args.cad.read_bytes()).hexdigest(),
    ground_truth_use="benchmark acquisition and evaluation only; no priors",
    object_belief_updates=False,
    motion="simulation arm only; collision checking enabled",
  )
  write_json(args.output / "run.json", metadata)
  write_json(args.output / "plan.json", plan)
  rows = []

  def save_rows():
    fields = sorted({key for row in rows for key in row})
    with (args.output / "results.csv").open("w", newline="") as stream:
      writer = csv.DictWriter(stream, fieldnames=fields)
      writer.writeheader()
      writer.writerows(rows)

  for view, target in zip(plan["views"], targets, strict=True):
    for repeat in range(args.repeats):
      require_idle_simulation(solution)
      base = dict(
        label=args.label,
        plan_id=plan_id,
        view_id=view["view_id"],
        repeat=repeat,
        azimuth_deg=view["azimuth_deg"],
        timestamp_utc=datetime.now(UTC).isoformat(),
      )
      directory = args.output / f"{view['view_id']}_{repeat:03d}"
      directory.mkdir()
      stage = "move"
      try:
        truth_before = actual_pose(simulator, plan["object_name"])
        drift = pose_errors(truth_before, center)
        if (
          drift["position_error_mm"] > 0.5 or drift["rotation_error_deg"] > 0.1
        ):
          raise RuntimeError(
            "Object differs from saved plan center; reset simulation externally before benchmarking."
          )
        quaternion = tuple(
          float(x)
          for x in rotation3.Rotation3.from_matrix(
            np.asarray(target.rotation).T
          ).quaternion.xyzw
        )
        task = robot.build_move_cartesian_task(
          "root",
          target_frame_offset=(target.translation, quaternion),
          motion_type="ANY",
        )
        solution.executive.run(task, blocking=True, silence_outputs=True)
        operation = solution.executive.operation
        if not operation.done or operation.proto.HasField("error"):
          raise RuntimeError(
            "Camera move failed; not continuing from an unknown arm state."
          )
        time.sleep(args.settle_seconds)
        stage = "capture"
        truth_before = actual_pose(simulator, plan["object_name"])
        capture = camera.capture(
          sensor_names=[rgb_name, depth_name], timeout=timedelta(seconds=10)
        )
        truth_after_capture = actual_pose(simulator, plan["object_name"])
        rgb = capture.sensor_images[rgb_name]
        depth = capture.sensor_images[depth_name]
        optical = sdk_transform(rgb.world_t_sensor)
        # Save native input before request preparation mutates sensor extrinsics.
        (directory / "capture.pb").write_bytes(
          capture.proto.SerializeToString()
        )
        np.save(directory / "depth.npy", depth.array, allow_pickle=False)
        Image.fromarray(rgb.array.astype(np.uint8)).save(directory / "rgb.png")
        write_json(
          directory / "capture.json",
          dict(
            capture_at=rgb.capture_at.isoformat(),
            world_t_optical=record(optical),
            intrinsics=rgb.intrinsic_matrix.tolist(),
            actual_before=record(truth_before),
            actual_after_capture=record(truth_after_capture),
            capture_metadata=json_format.MessageToDict(
              capture.proto, preserving_proto_field_name=True
            ),
          ),
        )
        stage = "inference"
        request = pose_estimation_client_utils.run_pose_estimation_request_from_capture_results(
          id_utils.id_proto_from_string(args.estimator),
          [capture],
          inference_timeout_secs=args.timeout,
        )
        (directory / "request.pb").write_bytes(request.SerializeToString())
        response = stub.RunPoseEstimation(request, timeout=args.timeout + 10)
        (directory / "response.pb").write_bytes(response.SerializeToString())
        write_json(
          directory / "response.json",
          json_format.MessageToDict(response, preserving_proto_field_name=True),
        )
        truth_after = actual_pose(simulator, plan["object_name"])
        drift = pose_errors(truth_after, truth_before)
        capture_drift = pose_errors(truth_after_capture, truth_before)
        stable = all(
          d["position_error_mm"] <= 0.5 and d["rotation_error_deg"] <= 0.1
          for d in (drift, capture_drift)
        )
        achieved = pose_errors(optical, transform(view["camera_pose"]))
        in_image = cad_in_image(
          rgb.array.shape, rgb.intrinsic_matrix, optical, truth_before, cad
        )
        common = dict(
          base,
          status="ok" if stable else "object_moved",
          stable=stable,
          cad_in_image=in_image,
          valid_for_comparison=stable
          and in_image
          and achieved["position_error_mm"] <= 2
          and achieved["rotation_error_deg"] <= 0.5,
          capture_at=rgb.capture_at.isoformat(),
          image=str(directory.relative_to(args.output) / "comparison.png"),
          detection_count=len(response.pose_estimates),
          achieved_camera_position_error_mm=achieved["position_error_mm"],
          achieved_camera_rotation_error_deg=achieved["rotation_error_deg"],
          **flatten_pose("actual", truth_before),
          **flatten_pose("camera", optical),
        )
        detections = []
        for index, estimate in enumerate(response.pose_estimates):
          pose = sdk_transform(
            proto_conversion.pose_from_proto(estimate.camera_t_target)
          )
          world_pose = (
            optical.compose(pose) if args.estimate_frame == "camera" else pose
          )
          detections.append(world_pose)
          rows.append(
            dict(
              common,
              detection_index=index,
              detection_id=estimate.id,
              score=estimate.score,
              **flatten_pose("detected", world_pose),
              **pose_errors(world_pose, truth_before),
            )
          )
        if not detections:
          rows.append(
            dict(common, status="no_detection" if stable else "object_moved")
          )
        comparison_image(
          rgb.array,
          rgb.intrinsic_matrix,
          optical,
          truth_before,
          detections,
          cad,
        ).save(directory / "comparison.png")
        write_json(directory / "actual_after.json", record(truth_after))
      except (
        grpc.RpcError,
        solutions_errors.Error,
        ValueError,
        RuntimeError,
        OSError,
      ) as error:
        rows.append(dict(base, status=f"{stage}_failed", error=str(error)))
        save_rows()
        if stage == "move":
          raise
      save_rows()
  print(
    f"Saved {len(rows)} rows to {args.output / 'results.csv'}. No pick or belief updates."
  )


def compare_runs(args):
  inputs = []
  for path in (args.baseline, args.corrected):
    with path.open(newline="") as stream:
      inputs.append(list(csv.DictReader(stream)))
  rows = pair_results(*inputs)
  fields = sorted({key for row in rows for key in row})
  with args.output.open("x", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)
  print(f"Saved {len(rows)} paired viewpoints to {args.output}.")


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  sub = parser.add_subparsers(dest="command", required=True)
  comparison = sub.add_parser("compare")
  comparison.add_argument("--baseline", type=Path, required=True)
  comparison.add_argument("--corrected", type=Path, required=True)
  comparison.add_argument("--output", type=Path, required=True)
  for command in ("plan", "run"):
    p = sub.add_parser(command)
    p.add_argument("--address", default="192.168.10.3:17080")
    p.add_argument("--plan", type=Path, required=True)
    if command == "plan":
      p.add_argument("--camera", default="orbbec_camera")
      p.add_argument("--object", default="raw_stock_2x3x5")
      p.add_argument("--rgb-sensor", default="1")
      p.add_argument("--depth-sensor", default="4")
      p.add_argument("--tool-object", default="gripper")
      p.add_argument("--tool-frame", default="tool_frame")
      p.add_argument("--views", type=int, default=12)
      p.add_argument("--radius", type=float)
      p.add_argument("--height", type=float)
    else:
      p.add_argument("--output", type=Path, required=True)
      p.add_argument("--cad", type=Path, required=True)
      p.add_argument("--provenance", type=Path, required=True)
      p.add_argument("--label", required=True)
      p.add_argument("--execute-simulation", action="store_true")
      p.add_argument("--arm-part", default="ur_module")
      p.add_argument("--service", default="pose_estimator_service")
      p.add_argument(
        "--estimator", default="ai.intrinsic.raw_stock_2x3x5_estimator"
      )
      p.add_argument(
        "--estimate-frame", choices=("camera", "root"), default="camera"
      )
      p.add_argument("--repeats", type=int, default=1)
      p.add_argument("--settle-seconds", type=float, default=1)
      p.add_argument("--timeout", type=int, default=30)
  args = parser.parse_args()
  if args.command == "run" and (
    args.repeats < 1
    or args.timeout < 1
    or not math.isfinite(args.settle_seconds)
    or args.settle_seconds < 0
  ):
    parser.error(
      "repeats/timeout must be positive; settle-seconds must be finite and nonnegative."
    )
  {"plan": create_plan, "run": collect, "compare": compare_runs}[args.command](
    args
  )


if __name__ == "__main__":
  main()
