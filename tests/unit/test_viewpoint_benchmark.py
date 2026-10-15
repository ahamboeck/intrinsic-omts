"""Repeatable camera orbit and pose-evaluation contracts."""

import math
import unittest
from types import SimpleNamespace

from src.grasping.geometry import IDENTITY, RigidTransform, dot, sub, unit
from tools.pose_estimation.run_viewpoint_benchmark import (
  collect,
  require_idle_simulation,
)
from tools.pose_estimation.viewpoint_benchmark import (
  make_plan,
  pair_results,
  pose_errors,
)


class ViewpointBenchmarkTest(unittest.TestCase):
  def test_motion_requires_explicit_acknowledgement(self):
    with self.assertRaisesRegex(ValueError, "execute-simulation"):
      collect(SimpleNamespace(execute_simulation=False))

  def test_hardware_and_busy_executive_rejected(self):
    with self.assertRaisesRegex(RuntimeError, "non-simulated"):
      require_idle_simulation(SimpleNamespace(is_simulated=False))
    with self.assertRaisesRegex(RuntimeError, "busy"):
      require_idle_simulation(
        SimpleNamespace(
          is_simulated=True,
          executive=SimpleNamespace(
            has_operation=True, operation=SimpleNamespace(done=False)
          ),
        )
      )

  def test_join_retains_failures_and_does_not_select_closest_detection(self):
    row = dict(plan_id="same", view_id="view_000", repeat="0", status="ok")
    output = pair_results(
      [dict(row, position_error_mm="83")], [dict(row, position_error_mm="1")]
    )
    self.assertEqual(output[0]["baseline_position_error_mm"], "83")
    self.assertEqual(output[0]["corrected_position_error_mm"], "1")
    output = pair_results([row, row], [])
    self.assertEqual(output[0]["baseline_status"], "ambiguous_multiple_rows")
    self.assertEqual(output[0]["corrected_status"], "missing")
    with self.assertRaises(ValueError):
      pair_results([row], [dict(row, plan_id="different")])

  def test_circle_is_world_horizontal_and_optical_z_looks_at_center(self):
    actual = RigidTransform(IDENTITY, (0.25, 0.25, 0.69))
    camera = RigidTransform(IDENTITY, (0.35, 0.25, 0.99))
    plan = make_plan(actual, camera, count=8)
    self.assertEqual(len(plan["views"]), 8)
    for view in plan["views"]:
      pose = RigidTransform(
        tuple(map(tuple, view["camera_pose"]["rotation_basis_columns"])),
        tuple(view["camera_pose"]["translation_m"]),
      )
      self.assertAlmostEqual(pose.translation[2], 0.99)
      self.assertAlmostEqual(
        math.hypot(*sub(pose.translation, actual.translation)[:2]), 0.1
      )
      self.assertAlmostEqual(
        dot(pose.rotation[2], unit(sub(actual.translation, pose.translation))),
        1,
      )
    self.assertEqual(
      plan["views"][0]["camera_pose"]["translation_m"], camera.translation
    )

  def test_exact_pose_and_known_translation_rotation_error(self):
    actual = RigidTransform()
    self.assertEqual(pose_errors(actual, actual)["rotation_error_deg"], 0)
    rotation = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    detected = RigidTransform(rotation, (0.003, 0.004, 0.0))
    errors = pose_errors(detected, actual)
    self.assertAlmostEqual(errors["position_error_mm"], 5)
    self.assertAlmostEqual(errors["rotation_error_deg"], 90)

  def test_cuboid_sign_symmetry_reported_separately(self):
    pose = RigidTransform(((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)))
    errors = pose_errors(pose, RigidTransform())
    self.assertEqual(errors["rotation_error_deg"], 180)
    self.assertEqual(errors["cuboid_symmetry_rotation_error_deg"], 0)

  def test_invalid_orbit_rejected(self):
    for kwargs in (
      dict(count=0),
      dict(radius=-1),
      dict(height=0),
      dict(radius=float("nan")),
    ):
      with self.assertRaises(ValueError):
        make_plan(
          RigidTransform(),
          RigidTransform(translation=(1.0, 0.0, 1.0)),
          **kwargs,
        )


if __name__ == "__main__":
  unittest.main()
