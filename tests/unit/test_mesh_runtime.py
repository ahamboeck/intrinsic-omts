"""Runtime frame script and its source bundle, without robot commands."""

import json
import math
import sys
import types
import unittest
from unittest import mock

import numpy as np
from intrinsic.math.python import data_types

from src.core.config import GraspConfig
from src.grasping.geometry import RigidTransform
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.mesh_inputs import cuboid_mesh
from src.grasping.runtime_frames import (
  contact_pair_diagnostics,
  from_sdk,
  plan_mesh_frames,
  to_sdk,
  top_down_direction_filter,
)
from src.grasping.types import ContactPair, SurfaceContact
from src.hardware.mesh_grasp_planner import runtime_source


class MeshRuntimeTest(unittest.TestCase):
  def test_diagnostics_measure_analytic_tilt_in_world_not_object_frame(self):
    pair = ContactPair(
      SurfaceContact((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), 0),
      SurfaceContact((0.05, 0.0, 0.0), (1.0, 0.0, 0.0), 1),
    )
    angle = math.radians(30)
    world = RigidTransform(
      (
        (math.cos(angle), 0.0, math.sin(angle)),
        (0.0, 1.0, 0.0),
        (-math.sin(angle), 0.0, math.cos(angle)),
      )
    )
    report = contact_pair_diagnostics(
      [pair], world, hande_collision_profile(), 0.06, 2
    )
    self.assertAlmostEqual(report["contact_width_mm"]["min"], 50)
    self.assertAlmostEqual(report["required_approach_tilt_deg"]["min"], 30)
    self.assertEqual(report["rejected_approach_tilt"], 1)
    self.assertEqual(report["rejected_open_width"], 0)
    predicate = top_down_direction_filter(world, 2)
    self.assertFalse(predicate((1.0, 0.0, 0.0)))
    self.assertFalse(predicate((-1.0, 0.0, 0.0)))
    self.assertTrue(predicate((0.0, 1.0, 0.0)))

  def test_empty_diagnostics_have_no_invented_minimum(self):
    report = contact_pair_diagnostics(
      [], RigidTransform(), hande_collision_profile(), 0.06, 2
    )
    self.assertIsNone(report["required_approach_tilt_deg"])
    self.assertIsNone(report["contact_width_mm"])

  def setUp(self):
    mesh = cuboid_mesh((0.0508, 0.127, 0.0762))
    self.params = types.SimpleNamespace(
      parent_object="root",
      workpiece_object_name="stock",
      cad_json=json.dumps(
        {"vertices": mesh.vertices, "triangles": mesh.triangles}
      ),
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      grasp_frame_name="grasp",
      pregrasp_frame_name="pre_grasp",
      pregrasp_distance_m=0.1,
      max_approach_tilt_deg=2,
    )
    self.world = mock.MagicMock()
    self.world.get_transform.return_value = data_types.Pose3()
    self.context = types.SimpleNamespace(object_world=self.world)

  def test_writes_only_locally_checked_frames(self):
    plan_mesh_frames(self.context, self.params)
    self.assertEqual(self.world.update_transform.call_count, 2)
    self.world.get_object.assert_called_once_with("stock")
    calls = self.world.update_transform.call_args_list
    grasp = from_sdk(calls[0].kwargs["a_t_b"])
    pregrasp = from_sdk(calls[1].kwargs["a_t_b"])
    self.assertAlmostEqual(
      math.dist(grasp.translation, pregrasp.translation), 0.1
    )

  def test_vertical_short_axis_rejects_without_stale_frame_updates(self):
    self.world.get_transform.return_value = data_types.Pose3(
      data_types.Rotation3.from_matrix(
        np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
      ),
      [0, 0, 0],
    )
    with self.assertRaisesRegex(ValueError, "No fallback"):
      plan_mesh_frames(self.context, self.params)
    self.world.update_transform.assert_not_called()

  def test_bundle_runs_without_omts_installed_in_executive(self):
    # Inspect actual emitted source and execute with no imports of src.grasping.
    source = runtime_source()
    self.assertNotIn("from src.grasping", source)
    with mock.patch.dict(sys.modules):
      exec(source, {"context": self.context, "params": self.params})
    self.assertEqual(self.world.update_transform.call_count, 2)

  def test_sdk_transform_round_trip(self):
    pose = data_types.Pose3(
      data_types.Rotation3.from_matrix(
        np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
      ),
      [1, 2, 3],
    )
    self.assertEqual(from_sdk(to_sdk(from_sdk(pose))), from_sdk(pose))

  def test_mesh_configuration_is_explicit_and_simulation_only(self):
    with self.assertRaisesRegex(ValueError, "simulation_only"):
      GraspConfig(planner="mesh", mesh_cad_json="fixture.json")
    with self.assertRaisesRegex(ValueError, "mesh_cad_json"):
      GraspConfig(planner="mesh", mesh_simulation_only=True)
    for limit in (True, math.inf, -1, 90):
      with self.subTest(limit=limit), self.assertRaises(ValueError):
        GraspConfig(
          planner="mesh",
          mesh_cad_json="fixture.json",
          mesh_simulation_only=True,
          mesh_max_approach_tilt_deg=limit,
        )


if __name__ == "__main__":
  unittest.main()
