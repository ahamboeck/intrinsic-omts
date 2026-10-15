"""Received-pose composition and ROS snapshot replacement contracts."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tools.grasping.pose_overlay import marker_specs, split_marker_specs
from tools.pose_estimation.pose_snapshot import make_snapshot, publish_snapshot

IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
CAD = dict(
  vertices=[[0, 0, 0], [0.01, 0, 0], [0, 0.02, 0]], triangles=[[0, 1, 2]]
)


def pose(translation, rotation=IDENTITY):
  return dict(translation_m=translation, rotation_basis_columns=rotation)


class PoseSnapshotTest(unittest.TestCase):
  def test_export_writes_complete_bundle_and_launches_native_watcher(self):
    native_popen = subprocess.Popen
    snapshot = make_snapshot(
      pose([0, 0, 0]), [], pose([1, 2, 3]), "now", "camera"
    )
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      cad = root / "cad.json"
      cad.write_text(json.dumps(CAD))
      manifest = root / "pixi.toml"
      manifest.write_text("")
      (root / "tools").mkdir()
      (root / "tools" / "__init__.py").write_text("")
      module = "tools.pose_estimation.pose_snapshot"
      with (
        mock.patch(module + "._SNAPSHOT_DIRECTORY", root),
        mock.patch(module + ".shutil.which", return_value=sys.executable),
        mock.patch(module + ".subprocess.Popen") as launch,
        mock.patch.dict(
          os.environ,
          {"PYTHONSAFEPATH": "1", "PYTHONPATH": "/bazel/runfiles"},
        ),
      ):
        path, _ = publish_snapshot(snapshot, cad, root, manifest)
        bundle = json.loads(path.read_text())
        self.assertEqual(bundle["cad"], CAD)
        self.assertEqual(bundle["finished_at_utc"], "now")
        self.assertEqual(launch.call_args.args[0][-2:], ["--watch", str(path)])
        self.assertEqual(launch.call_args.kwargs["cwd"], root)
        # The native Python must find workspace modules even when Bazel
        # disables automatic insertion of the current directory into sys.path.
        with mock.patch("subprocess.Popen", native_popen):
          child = subprocess.run(
            [sys.executable, "-c", "import tools"],
            cwd=root,
            env=launch.call_args.kwargs["env"],
            capture_output=True,
            text=True,
          )
        self.assertEqual(child.returncode, 0, child.stderr)
        # Invalid new geometry must not replace the previous valid snapshot.
        before = path.read_bytes()
        cad.write_text("{}")
        with self.assertRaises(KeyError):
          publish_snapshot(snapshot, cad, root, manifest)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(launch.call_count, 1)

  def test_received_camera_pose_composed_not_replaced_by_actual(self):
    camera = pose([1, 2, 3], [[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    received = [dict(id="part", score=0.7, pose=pose([0.1, 0.2, 0.3]))]
    actual = pose([8, 9, 10])
    snapshot = make_snapshot(camera, received, actual, "now", "camera")
    perceived = snapshot["perceived_objects"][0]
    self.assertEqual(perceived["pose"]["translation_m"], (0.8, 2.1, 3.3))
    self.assertEqual(
      perceived["pose"]["rotation_basis_columns"],
      tuple(map(tuple, camera["rotation_basis_columns"])),
    )
    self.assertEqual(perceived["score"], 0.7)
    self.assertEqual(snapshot["actual_after_evaluation_only"], actual)
    groups = split_marker_specs(marker_specs(CAD, snapshot))
    self.assertEqual(groups["actual"][0]["points"][0], (8, 9, 10))
    self.assertEqual(groups["perceived"][0]["points"][0], (0.8, 2.1, 3.3))

  def test_root_result_is_not_transformed_twice_and_all_detections_retained(
    self,
  ):
    estimates = [dict(id=str(i), pose=pose([i, 0, 0])) for i in range(2)]
    snapshot = make_snapshot(pose([9, 9, 9]), estimates, None, "now", "root")
    groups = split_marker_specs(marker_specs(CAD, snapshot))
    self.assertEqual(groups["actual"], [])
    self.assertEqual(len(groups["perceived"]), 10)
    self.assertEqual({s["id"] for s in groups["perceived"]}, set(range(10)))
    self.assertEqual(groups["perceived"][5]["points"][0], (1, 0, 0))

  def test_empty_detection_snapshot_has_no_perceived_markers(self):
    snapshot = make_snapshot(
      pose([0, 0, 0]), [], pose([0, 0, 0]), "now", "camera"
    )
    groups = split_marker_specs(marker_specs(CAD, snapshot))
    self.assertEqual(len(groups["actual"]), 5)
    self.assertEqual(groups["perceived"], [])

  def test_unknown_frame_is_rejected(self):
    with self.assertRaises(ValueError):
      make_snapshot(pose([0, 0, 0]), [], None, "now", "unknown")


if __name__ == "__main__":
  unittest.main()
