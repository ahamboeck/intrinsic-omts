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

"""Tests for the cuboid-center grasp planner."""

import math
from typing import Any
from unittest import mock

from absl.testing import absltest
from intrinsic.math.python import data_types
from intrinsic.solutions import behavior_tree as bt

from src.core.config import load_app_config
from src.grasping.interface import Approach
from src.grasping.planners.cuboid_center import adapter
from src.grasping.planners.cuboid_center.algorithm import (
  plan_cuboid_center_grasp,
)
from src.utils.script_utils import load_python_script


def _make_params(**overrides: Any) -> Any:
  """Creates a parameter object mimicking the script's params proto."""
  defaults = {
    "parent_object": "root",
    "workpiece_object_name": "raw_stock_2x3x5",
    "tool_object_name": "gripper",
    "tool_frame_name": "tool_frame",
    "approach_offset_z": 0.08,
    "pregrasp_frame_name": "pre_grasp",
    "grasp_frame_name": "grasp",
  }
  defaults.update(overrides)
  params = mock.MagicMock()
  for key, val in defaults.items():
    setattr(params, key, val)
  return params


def _world(
  workpiece_pose: data_types.Pose3,
  tool_pose: data_types.Pose3 | None = None,
  existing_frames: tuple[str, ...] = (),
) -> mock.MagicMock:
  """Returns a mock world holding the workpiece and, optionally, the tool."""
  world = mock.MagicMock()
  world.root.list_frames.return_value = list(existing_frames)

  def _get_transform(node_a: Any, node_b: Any) -> data_types.Pose3 | None:
    del node_a
    if node_b is world.raw_stock_2x3x5:
      return workpiece_pose
    if node_b is world.gripper.tool_frame:
      return tool_pose
    return None

  world.get_transform.side_effect = _get_transform
  return world


def _created(world: mock.MagicMock) -> dict[str, data_types.Pose3]:
  return {
    call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
    for call in world.create_frame.call_args_list
  }


def _pose(xyz, xyzw=(0.0, 0.0, 0.0, 1.0)) -> data_types.Pose3:
  return data_types.Pose3(
    data_types.Rotation3(data_types.Quaternion(list(xyzw))), list(xyz)
  )


class CuboidCenterAlgorithmTest(absltest.TestCase):
  """Tests the grasp the script writes for a workpiece pose."""

  def test_runs_via_load_python_script_in_isolated_namespace(self) -> None:
    """The script must run hermetically, as `bt.PythonScript` runs it."""
    world = _world(_pose((0.3, -0.2, 1.02)))
    exec(  # noqa: S102
      load_python_script(plan_cuboid_center_grasp),
      {"context": mock.Mock(object_world=world), "params": _make_params()},
    )
    self.assertEqual(set(_created(world)), {"pre_grasp", "grasp"})

  def test_grasps_the_workpiece_center_with_the_approach_offset(self) -> None:
    world = _world(_pose((0.15, 0.15, 1.05)))
    plan_cuboid_center_grasp(
      mock.Mock(object_world=world), _make_params(approach_offset_z=0.08)
    )

    created = _created(world)
    grasp, pregrasp = created["grasp"], created["pre_grasp"]
    for actual, expected in zip(
      grasp.translation, (0.15, 0.15, 1.05), strict=True
    ):
      self.assertAlmostEqual(float(actual), expected, places=5)
    for actual, expected in zip(
      pregrasp.translation, (0.15, 0.15, 1.13), strict=True
    ):
      self.assertAlmostEqual(float(actual), expected, places=5)

  def test_updates_existing_frames(self) -> None:
    world = _world(
      _pose((0.15, 0.15, 1.0)), existing_frames=("pre_grasp", "grasp")
    )
    plan_cuboid_center_grasp(mock.Mock(object_world=world), _make_params())

    world.create_frame.assert_not_called()
    self.assertEqual(world.update_transform.call_count, 2)

  def test_upright_workpiece_falls_back_to_medium_axis_az(self) -> None:
    """When local X points along root Z, alignment uses local Z."""
    # Pitch -90 deg around Y: local X -> root +Z, local Z -> root -X (psi = pi).
    half_angle = -math.pi / 4.0
    world = _world(
      _pose(
        (0.3, -0.2, 1.0),
        (0.0, math.sin(half_angle), 0.0, math.cos(half_angle)),
      )
    )
    plan_cuboid_center_grasp(mock.Mock(object_world=world), _make_params())

    # With psi = pi, half_psi = pi/2, c1 = (0, 1, 0, 0).
    q = _created(world)["grasp"].rotation.quaternion
    self.assertAlmostEqual(abs(float(q.y)), 1.0, places=5)
    self.assertAlmostEqual(float(q.x), 0.0, places=5)

  def test_picks_the_orientation_closest_to_the_current_tool(self) -> None:
    # Identity workpiece rotation gives the candidates (+-1, 0, 0, 0) and
    # (0, +-1, 0, 0); the tool is at -c2 = (0, -1, 0, 0).
    world = _world(
      _pose((0.3, -0.2, 1.0)),
      tool_pose=_pose((0.3, -0.2, 1.1), (0.0, -1.0, 0.0, 0.0)),
    )
    plan_cuboid_center_grasp(mock.Mock(object_world=world), _make_params())

    q = _created(world)["grasp"].rotation.quaternion
    self.assertAlmostEqual(float(q.x), 0.0, places=5)
    self.assertAlmostEqual(float(q.y), -1.0, places=5)
    self.assertAlmostEqual(float(q.z), 0.0, places=5)
    self.assertAlmostEqual(float(q.w), 0.0, places=5)

  def test_missing_workpiece_fails_loudly(self) -> None:
    world = _world(_pose((0.3, -0.2, 1.0)))
    world.raw_stock_2x3x5 = None
    with self.assertRaisesRegex(ValueError, "raw_stock_2x3x5"):
      plan_cuboid_center_grasp(mock.Mock(object_world=world), _make_params())


class CuboidCenterAdapterTest(absltest.TestCase):
  def test_is_the_touchdown_default(self) -> None:
    planner = adapter.CuboidCenterGraspPlanner(
      mock.MagicMock(), "gripper", "tool_frame", 0.08
    )
    self.assertFalse(planner.execution.open_before_planning)
    self.assertIs(planner.execution.approach, Approach.TOUCHDOWN)
    self.assertFalse(planner.execution.replan_at_unload)
    self.assertFalse(planner.simulation_only)

  def test_builds_the_script_task_from_the_cell_config(self) -> None:
    config = load_app_config("configs/omts/app_config.yaml")
    with mock.patch.object(
      adapter, "create_python_script", wraps=adapter.create_python_script
    ) as script:
      planner = adapter.create(mock.sentinel.solution, config, None)
      task = planner.build_plan_grasp_task(
        "raw_stock_2x3x5", "root", "grasp_unload", "pre_grasp_unload"
      )

    self.assertIsInstance(task, bt.Task)
    solution, function, fields = script.call_args.args
    self.assertIs(solution, mock.sentinel.solution)
    self.assertIs(function, plan_cuboid_center_grasp)
    self.assertEqual(
      dict((name, value) for _, name, value in fields),
      {
        "parent_object": "root",
        "workpiece_object_name": "raw_stock_2x3x5",
        "tool_object_name": "gripper",
        "tool_frame_name": "tool_frame",
        "approach_offset_z": config.cycle.approach_offset_z,
        "grasp_frame_name": "grasp_unload",
        "pregrasp_frame_name": "pre_grasp_unload",
      },
    )


if __name__ == "__main__":
  absltest.main()
