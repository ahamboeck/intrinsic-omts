"""Offline geometry bridge and bounded selection; no runtime success claims."""

import math
import unittest
from dataclasses import replace

from src.grasping.geometry import RigidTransform
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.mesh_inputs import cuboid_mesh, normalize_mesh
from src.grasping.profile import CollisionBox
from src.grasping.selection import (
  FeasibilityResult,
  SelectionOptions,
  select_grasp,
)
from src.grasping.types import ContactPairOptions, TriangleMesh


class MeshInputsTest(unittest.TestCase):
  def test_full_cuboid_dimensions_and_outward_faces(self):
    mesh = cuboid_mesh((0.02, 0.03, 0.04))
    self.assertEqual(len(mesh.vertices), 8)
    self.assertEqual(len(mesh.triangles), 12)
    self.assertEqual(mesh.vertices[0], (-0.01, -0.015, -0.02))
    # Face normal points away from the origin, independent of search code.
    for face in mesh.triangles:
      a, b, c = (mesh.vertices[i] for i in face)
      u = tuple(b[i] - a[i] for i in range(3))
      v = tuple(c[i] - a[i] for i in range(3))
      n = (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
      )
      self.assertGreater(sum(n[i] * a[i] for i in range(3)), 0)

  def test_units_then_mesh_to_entity_transform(self):
    source = cuboid_mesh((20, 30, 40))
    transform = RigidTransform(((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (1, 2, 3))
    mesh = normalize_mesh(
      source, metres_per_unit=0.001, object_T_mesh=transform
    )
    self.assertEqual(mesh.triangles, source.triangles)
    for expected, actual in zip(
      (1.015, 1.99, 2.98), mesh.vertices[0], strict=True
    ):
      self.assertAlmostEqual(actual, expected)

  def test_invalid_dimensions_and_scale_fail(self):
    for dimensions in ((0, 1, 1), (1, -1, 1), (math.nan, 1, 1), (True, 1, 1)):
      with self.subTest(dimensions=dimensions), self.assertRaises(ValueError):
        cuboid_mesh(dimensions)
    for scale in (0, -1, math.nan, math.inf, True):
      with self.subTest(scale=scale), self.assertRaises(ValueError):
        normalize_mesh(
          cuboid_mesh((1, 1, 1)),
          metres_per_unit=scale,
          object_T_mesh=RigidTransform(),
        )


class SelectionTest(unittest.TestCase):
  def setUp(self):
    self.mesh = cuboid_mesh((0.02, 0.02, 0.02))
    self.profile = hande_collision_profile()
    self.options = SelectionOptions(
      contact_options=ContactPairOptions(
        seed=42, max_pairs=20, opening_clearance_m=0.001
      ),
      pregrasp_distance_m=0.08,
      open_width_m=0.06,
      max_feasibility_checks=8,
    )

  def run_selection(self, check, **kwargs):
    return select_grasp(
      self.mesh,
      self.profile,
      world_T_object=RigidTransform(translation=(1, 2, 3)),
      options=kwargs.pop("options", self.options),
      check_feasibility=check,
      deadline=10,
      clock=lambda: 1,
      **kwargs,
    )

  def test_rejected_first_candidate_then_accept_second(self):
    calls = []

    def check(grasp, pregrasp, remaining):
      calls.append((grasp, pregrasp, remaining))
      return FeasibilityResult(
        len(calls) == 2, "blocked approach" if len(calls) == 1 else ""
      )

    report = self.run_selection(check)
    self.assertIsNotNone(report.selected)
    self.assertEqual(report.feasibility_checks, 2)
    self.assertEqual(report.rejections[-1].reason, "blocked approach")
    grasp, pregrasp, remaining = calls[-1]
    self.assertEqual(remaining, 9)
    self.assertAlmostEqual(pregrasp.translation[2] - grasp.translation[2], 0.08)
    self.assertAlmostEqual(
      grasp.translation[0], 1 + report.selected.object_T_tcp.translation[0]
    )

  def test_deterministic_ranking(self):
    def check(*args):
      return FeasibilityResult(True)

    self.assertEqual(self.run_selection(check), self.run_selection(check))

  def test_no_feasible_candidate_has_no_selected_pose(self):
    report = self.run_selection(
      lambda *args: FeasibilityResult(False, "unreachable")
    )
    self.assertIsNone(report.selected)
    self.assertLessEqual(report.feasibility_checks, 8)
    self.assertTrue(report.rejections)

  def test_feasibility_budget_is_enforced(self):
    calls = []

    def check(*args):
      calls.append(args)
      return FeasibilityResult(False, "blocked")

    report = self.run_selection(
      check, options=replace(self.options, max_feasibility_checks=1)
    )
    self.assertEqual(len(calls), 1)
    self.assertEqual(report.feasibility_checks, 1)
    self.assertIsNone(report.selected)
    self.assertEqual(report.rejections[-1].stage, "budget")

  def test_dedup_preserves_distinct_yaw_alternatives(self):
    report = self.run_selection(
      lambda *args: FeasibilityResult(False, "blocked"),
      options=replace(self.options, position_tolerance_m=1),
    )
    # Centered cube has +/-X and +/-Y jaw orientations. Position-only dedup
    # would incorrectly throw away three of these four orientations.
    self.assertEqual(report.feasibility_checks, 4)
    self.assertTrue(any(r.stage == "duplicate" for r in report.rejections))

  def test_open_shell_is_rejected_not_replaced_by_a_box(self):
    self.mesh = TriangleMesh(self.mesh.vertices, self.mesh.triangles[:1])
    with self.assertRaises(ValueError):
      self.run_selection(lambda *args: FeasibilityResult(True))

  def test_local_collision_rejection_never_calls_downstream(self):
    self.profile = replace(
      self.profile,
      body_boxes=(CollisionBox("blocking_body", RigidTransform(), (1, 1, 1)),),
    )

    def forbidden(*args):
      self.fail("downstream called for locally colliding candidate")

    report = self.run_selection(forbidden)
    self.assertIsNone(report.selected)
    self.assertEqual(report.feasibility_checks, 0)
    self.assertGreater(report.pose_proposals, 0)
    self.assertTrue(report.rejections)

  def test_too_wide_object_does_not_call_downstream(self):
    self.mesh = cuboid_mesh((0.1, 0.1, 0.1))

    def forbidden(*args):
      self.fail("downstream called for a too-wide object")

    report = self.run_selection(forbidden)
    self.assertIsNone(report.selected)
    self.assertEqual(report.contact_pairs, 0)

  def test_timeout_during_callback_never_accepts_result(self):
    now = [1]

    def check(*args):
      now[0] = 11
      return FeasibilityResult(True)

    with self.assertRaises(TimeoutError):
      select_grasp(
        self.mesh,
        self.profile,
        world_T_object=RigidTransform(),
        options=self.options,
        check_feasibility=check,
        deadline=10,
        clock=lambda: now[0],
      )

  def test_expired_deadline_and_malformed_callback_fail(self):
    with self.assertRaises(TimeoutError):
      select_grasp(
        self.mesh,
        self.profile,
        world_T_object=RigidTransform(),
        options=self.options,
        check_feasibility=lambda *args: True,
        deadline=0,
        clock=lambda: 1,
      )
    with self.assertRaises(ValueError):
      self.run_selection(lambda *args: True)

  def test_mesh_budget_and_invalid_options(self):
    with self.assertRaises(ValueError):
      self.run_selection(
        lambda *args: FeasibilityResult(True),
        options=replace(self.options, max_triangles=1),
      )
    for key, value in (
      ("max_feasibility_checks", True),
      ("position_tolerance_m", math.nan),
      ("rotation_tolerance_rad", math.pi),
      ("open_width_m", 0),
    ):
      with self.subTest(key=key), self.assertRaises(ValueError):
        replace(self.options, **{key: value})


if __name__ == "__main__":
  unittest.main()
