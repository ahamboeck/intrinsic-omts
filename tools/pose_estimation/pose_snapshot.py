"""Fresh perception-to-RViz snapshot contract, independent of ROS and the SDK."""

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from src.grasping.geometry import RigidTransform
from tools.grasping.pose_overlay import marker_specs

_SNAPSHOT_DIRECTORY = Path("/tmp/opencode")


def pose_record(pose):
  return dict(
    translation_m=pose.translation, rotation_basis_columns=pose.rotation
  )


def transform_from_record(value):
  return RigidTransform(
    tuple(tuple(axis) for axis in value["rotation_basis_columns"]),
    tuple(value["translation_m"]),
  )


def make_snapshot(camera_pose, estimates, actual_pose, timestamp, frame):
  """No belief-pose substitution; map received camera poses into world metres."""
  if frame not in ("camera", "root"):
    raise ValueError("Estimate frame must be camera or root.")
  camera = transform_from_record(camera_pose)
  perceived = []
  for estimate in estimates:
    pose = transform_from_record(estimate["pose"])
    if frame == "camera":
      pose = camera.compose(pose)
    perceived.append(dict(estimate, pose=pose_record(pose)))
  return dict(
    finished_at_utc=timestamp,
    captured_world_camera=camera_pose,
    perceived_objects=perceived,
    actual_after_evaluation_only=actual_pose,
    estimate_frame=frame,
    simulator_state_use="evaluation only, never estimator or planner input",
  )


def publish_snapshot(snapshot, cad_path, workspace, manifest=None):
  """Atomically update one host snapshot; a singleton ROS process watches it."""
  workspace = Path(workspace)
  manifest = (
    Path(manifest)
    if manifest
    else (workspace.parent.parent / "experiments/native_viewers/pixi.toml")
  )
  pixi = shutil.which("pixi") or str(Path.home() / ".pixi/bin/pixi")
  if not manifest.is_file() or not Path(pixi).is_file():
    raise RuntimeError(
      "Native ROS Pixi environment unavailable; set --overlay_ros_manifest."
    )
  bundle = dict(snapshot, cad=json.loads(Path(cad_path).read_text()))
  # Validate geometry and poses before replacing a known-good snapshot.
  marker_specs(bundle["cad"], bundle)
  directory = _SNAPSHOT_DIRECTORY
  directory.mkdir(exist_ok=True)
  target = directory / "omts_perception_overlay.json"
  fd, temporary = tempfile.mkstemp(dir=directory, suffix=".json")
  try:
    with os.fdopen(fd, "w") as stream:
      json.dump(bundle, stream, allow_nan=False)
    os.replace(temporary, target)
  finally:
    if os.path.exists(temporary):
      os.unlink(temporary)
  with (directory / "omts_perception_overlay.log").open("a") as log:
    process = subprocess.Popen(
      [
        pixi,
        "run",
        "--manifest-path",
        str(manifest.resolve()),
        "--frozen",
        "-e",
        "ros",
        "python",
        "-B",
        "-m",
        "tools.grasping.pose_overlay",
        "--watch",
        str(target),
      ],
      cwd=workspace,
      stdin=subprocess.DEVNULL,
      stdout=log,
      stderr=subprocess.STDOUT,
      start_new_session=True,
      # Bazel sets PYTHONSAFEPATH=1. Keep that isolation, but explicitly make
      # workspace modules available to the external ROS interpreter instead
      # of inheriting Bazel's runfiles PYTHONPATH or relying on cwd insertion.
      env={
        **{
          k: v
          for k, v in os.environ.items()
          if k not in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_PRELOAD")
        },
        "PYTHONPATH": str(workspace.resolve()),
      },
    )
  return target, process
