# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the suction mesh grasp planner, its skill and its candidates."""

import dataclasses
import math
import types
from unittest import mock

import numpy as np
from absl.testing import absltest
from intrinsic.skills.python import skill_interface

from src.core.config import load_app_config
from src.grasping import gripper_model_pb2, workpiece_mesh
from src.grasping.interface import SUCTION, Approach
from src.grasping.planners.suction_mesh import adapter, candidates
from src.grasping.planners.suction_mesh.skill import (
  suction_mesh_plan_grasp,
  suction_mesh_plan_grasp_pb2,
)
from tests.unit import mesh_skill_fakes

_CUP = gripper_model_pb2.SuctionGripper(
  cup_diameter_m=0.02, contact_depth_m=0.0
)

# Outward-facing triangles of the unit cube [0, 1]^3.
_CUBE_TRIANGLES = (
  (0, 2, 1),
  (0, 3, 2),
  (4, 5, 6),
  (4, 6, 7),
  (0, 1, 5),
  (0, 5, 4),
  (1, 2, 6),
  (1, 6, 5),
  (2, 3, 7),
  (2, 7, 6),
  (3, 0, 4),
  (3, 4, 7),
)


def _boxes(*boxes):
  """Returns (vertices, triangles) for boxes given as (min corner, size)."""
  vertices, triangles = [], []
  for low, size in boxes:
    base = len(vertices)
    for corner in (
      (0, 0, 0),
      (1, 0, 0),
      (1, 1, 0),
      (0, 1, 0),
      (0, 0, 1),
      (1, 0, 1),
      (1, 1, 1),
      (0, 1, 1),
    ):
      vertices.append([low[i] + corner[i] * size[i] for i in range(3)])
    triangles.extend([base + i for i in tri] for tri in _CUBE_TRIANGLES)
  return workpiece_mesh.validate_mesh(vertices, triangles)


def _centered_box(size):
  return _boxes(([-s / 2 for s in size], size))


def _pose(z=0.0, roll_deg=0.0):
  pose = np.eye(4)
  c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
  pose[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
  pose[2, 3] = z
  return pose


class TopDownSuctionGraspsTest(absltest.TestCase):
  def test_box_is_grasped_at_the_center_of_its_top(self):
    found = candidates.top_down_grasps(
      *_centered_box((0.06, 0.1, 0.03)), _pose(z=0.7), _CUP
    )

    self.assertNotEmpty(found)
    best = found[0]
    np.testing.assert_allclose(best.grasp[:2, 3], [0.0, 0.0], atol=0.003)
    # Tool z points down; the cup presses 1 mm into the top at z 0.715.
    np.testing.assert_allclose(best.grasp[:3, 2], [0, 0, -1])
    self.assertAlmostEqual(best.grasp[2, 3], 0.714)
    self.assertAlmostEqual(best.hover[2, 3], 0.719)
    self.assertAlmostEqual(best.pregrasp[2, 3], 0.814)
    self.assertAlmostEqual(best.flatness_m, 0.0)

  def test_tool_frame_offset_from_the_cup_lip_is_applied(self):
    cup = gripper_model_pb2.SuctionGripper(
      cup_diameter_m=0.02, contact_depth_m=0.05
    )
    (best, *_) = candidates.top_down_grasps(
      *_centered_box((0.06, 0.1, 0.03)), _pose(), cup
    )
    self.assertAlmostEqual(best.grasp[2, 3], 0.015 - 0.001 + 0.05)

  def test_cup_overhanging_the_part_is_rejected(self):
    narrow = _centered_box((0.015, 0.1, 0.03))
    self.assertEmpty(candidates.top_down_grasps(*narrow, _pose(), _CUP))

  def test_cup_across_a_step_is_rejected(self):
    # Two plates 2 mm apart whose tops differ by 10 mm: only spots wholly on
    # one plate seal. (Touching boxes would not form a valid closed mesh.)
    stepped = _boxes(
      ((0.0, 0.0, 0.0), (0.04, 0.1, 0.02)),
      ((0.042, 0.0, 0.0), (0.04, 0.1, 0.03)),
    )
    found = candidates.top_down_grasps(*stepped, _pose(), _CUP)

    self.assertNotEmpty(found)
    for candidate in found:
      x = candidate.grasp[0, 3]
      self.assertTrue(x <= 0.03 + 1e-9 or x >= 0.052 - 1e-9, x)

  def test_tilted_top_beyond_the_limit_is_rejected(self):
    box = _centered_box((0.06, 0.1, 0.03))
    self.assertNotEmpty(
      candidates.top_down_grasps(*box, _pose(roll_deg=5), _CUP)
    )
    self.assertEmpty(candidates.top_down_grasps(*box, _pose(roll_deg=20), _CUP))

  def test_tool_x_follows_the_preferred_axis(self):
    (best, *_) = candidates.top_down_grasps(
      *_centered_box((0.06, 0.1, 0.03)),
      _pose(),
      _CUP,
      preferred_x_axis=(0.0, 1.0, 0.3),
    )
    np.testing.assert_allclose(best.grasp[:3, 0], [0, 1, 0], atol=1e-9)
    np.testing.assert_allclose(
      np.cross(best.grasp[:3, 0], best.grasp[:3, 1]), best.grasp[:3, 2]
    )


def _execute(planner, *, world=None, gripper=_CUP):
  world = world or mesh_skill_fakes.World()
  params = suction_mesh_plan_grasp_pb2.SuctionMeshPlanGraspParams(
    workpiece_object="box",
    robot_object="ur_module",
    tool_object="gripper",
    tool_frame="tool_frame",
    grasp_frame="grasp",
    pregrasp_frame="pre_grasp",
  )
  if gripper is not None:
    params.gripper.CopyFrom(gripper)
  context = types.SimpleNamespace(
    object_world=world, motion_planner=planner, geometry_service=None
  )
  return suction_mesh_plan_grasp.SuctionMeshPlanGrasp().execute(
    types.SimpleNamespace(params=params), context
  ), world


class SuctionMeshPlanGraspSkillTest(absltest.TestCase):
  def test_writes_a_grasp_on_the_box_top(self):
    result, world = _execute(mesh_skill_fakes.Planner())

    self.assertEqual(result.num_checked, 1)
    self.assertCountEqual(world.updates, ["grasp", "pre_grasp"])
    grasp = world.updates["grasp"]
    # The fake box is 30 mm tall at z 0.7, so its top is at 0.715.
    np.testing.assert_allclose(grasp[:3, 2], [0, 0, -1], atol=1e-9)
    self.assertAlmostEqual(grasp[2, 3], 0.714)
    np.testing.assert_allclose(grasp[:2, 3], [0.3, 0.2], atol=0.003)

  def test_checks_the_approach_to_just_above_the_surface(self):
    planner = mesh_skill_fakes.Planner()
    _, world = _execute(planner)

    (_, specification), *_ = planner.paths
    target = specification.motion_segments[0].target.cartesian_pose
    # The cup must not touch the part during the collision check.
    self.assertAlmostEqual(target.target_frame_offset.position.z, 0.719)
    self.assertGreater(
      target.target_frame_offset.position.z, world.updates["grasp"][2, 3]
    )

  def test_skips_spots_that_cannot_be_reached(self):
    result, _ = _execute(mesh_skill_fakes.Planner(ik_failures=1))
    self.assertEqual(result.num_checked, 2)

  def test_part_without_a_flat_spot_fails_clearly(self):
    big_cup = gripper_model_pb2.SuctionGripper(cup_diameter_m=0.05)
    with self.assertRaisesRegex(skill_interface.SkillError, "50 mm"):
      _execute(mesh_skill_fakes.Planner(), gripper=big_cup)

  def test_requires_the_cup_geometry(self):
    with self.assertRaises(skill_interface.InvalidSkillParametersError):
      _execute(mesh_skill_fakes.Planner(), gripper=None)


class SuctionMeshGraspPlannerTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.config = load_app_config("configs/lab_bb_01/app_config_suction.yaml")

  def test_is_a_simulation_only_suction_planner_that_replans(self):
    planner = adapter.SuctionMeshGraspPlanner
    self.assertEqual(planner.gripper_kinds, frozenset({SUCTION}))
    self.assertTrue(planner.simulation_only)
    self.assertFalse(planner.execution.open_before_planning)
    self.assertIs(planner.execution.approach, Approach.DIRECT)
    self.assertTrue(planner.execution.replan_at_unload)

  def test_requires_a_suction_model(self):
    hande = load_app_config("configs/lab_bb_01/app_config.yaml")
    with self.assertRaisesRegex(ValueError, "gripper.model.suction"):
      adapter.create(mock.MagicMock(), hande, adapter.SuctionMeshSettings())

  def test_rejects_invalid_settings(self):
    with self.assertRaisesRegex(ValueError, "pregrasp_distance_m"):
      adapter.SuctionMeshSettings(pregrasp_distance_m=0.0)
    with self.assertRaisesRegex(ValueError, "flatness_tolerance_m"):
      adapter.SuctionMeshSettings(flatness_tolerance_m=-0.001)
    with self.assertRaisesRegex(ValueError, "press_depth_m"):
      adapter.SuctionMeshSettings(press_depth_m=0.02)

  @mock.patch.object(adapter.bt, "Task")
  def test_builds_the_skill_task_with_the_cell_cup(self, unused_task):
    solution = mock.MagicMock()
    settings = dataclasses.replace(
      adapter.SuctionMeshSettings(), flatness_tolerance_m=0.003
    )
    planner = adapter.create(solution, self.config, settings)

    planner.build_plan_grasp_task("bracket", grasp_frame_name="grasp_unload")

    kwargs = (
      solution.skills.ai.intrinsic.suction_mesh_plan_grasp.call_args.kwargs
    )
    self.assertEqual(kwargs["workpiece_object"], "bracket")
    self.assertEqual(kwargs["grasp_frame"], "grasp_unload")
    self.assertEqual(kwargs["flatness_tolerance_m"], 0.003)
    self.assertEqual(kwargs["gripper"], self.config.gripper.model.suction)


if __name__ == "__main__":
  absltest.main()
