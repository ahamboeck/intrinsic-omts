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

"""Unit tests for world-updating behavior tree scripts and math_utils."""

import math
from typing import Any
from unittest import mock

from absl.testing import absltest
from intrinsic.math.python import data_types

from src.utils.math_utils import (
  create_transform_node_ref,
  describe_motion_types,
  normalize_angle,
  normalize_joint_angles,
  normalize_motion_types,
  object_exists_in_world,
)
from src.utils.random_placement import randomize_placement_frame
from src.utils.regrasp_frames import (
  save_workpiece_at_grasp,
  update_regrasp_return_frames,
)
from src.utils.script_utils import load_python_script
from src.utils.workpiece_pose import update_detected_workpiece_pose


def _make_params(**overrides: Any) -> Any:
  """Creates a parameter object mimicking SBL PythonScript params proto."""
  defaults = {
    "parent_object": "root",
    "camera_name": "orbbec_camera",
    "pos_x": 0.30,
    "pos_y": -0.20,
    "pos_z": 1.05,
    "ori_x": 0.0,
    "ori_y": 0.0,
    "ori_z": 0.0,
    "ori_w": 1.0,
    "target_scene_object_id": "ai.intrinsic.raw_stock_2x3x5",
    "min_safe_z": 0.95,
  }
  defaults.update(overrides)
  params = mock.MagicMock()
  for key, val in defaults.items():
    setattr(params, key, val)
  return params


def _world_with_stock() -> tuple[mock.MagicMock, mock.MagicMock]:
  """Returns a mock world holding `raw_stock_2x3x5` under root, and the stock."""
  world = mock.MagicMock()
  world.get_transform.return_value = None
  stock = mock.MagicMock()
  stock.parent = world.root
  # A bare MagicMock would auto-create the dotted id and satisfy the first
  # lookup, so null it out to exercise the short-name fallback as the real
  # world does.
  setattr(world, "ai.intrinsic.raw_stock_2x3x5", None)
  world.raw_stock_2x3x5 = stock
  return world, stock


class WorkpiecePoseTest(absltest.TestCase):
  """Tests writing the detected workpiece pose into the world."""

  def test_exec_via_load_python_script_in_isolated_namespace(self) -> None:
    """Verifies the script executes hermetically via exec() like bt.PythonScript."""
    world, stock = _world_with_stock()
    exec(  # noqa: S102
      load_python_script(update_detected_workpiece_pose),
      {
        "context": mock.Mock(object_world=world),
        "params": _make_params(pos_z=1.02),
      },
    )
    world.update_transform.assert_called_once()
    self.assertIs(world.update_transform.call_args.kwargs["node_b"], stock)

  def test_composes_the_camera_pose(self) -> None:
    """Verifies root_t_target = root_t_camera * cam_pose."""
    world, _ = _world_with_stock()
    camera = mock.MagicMock()
    world.orbbec_camera = camera
    # Camera mounted at (0.1, 0.2, 1.5) in root with identity rotation.
    root_t_camera = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0])),
      [0.1, 0.2, 1.5],
    )
    world.get_transform.side_effect = lambda a, b: (
      root_t_camera if b is camera.sensor else None
    )
    # Part at (0.05, -0.05, -0.45) in camera frame -> (0.15, 0.15, 1.05).
    params = _make_params(pos_x=0.05, pos_y=-0.05, pos_z=-0.45)

    update_detected_workpiece_pose(mock.Mock(object_world=world), params)

    pose = world.update_transform.call_args.kwargs["a_t_b"]
    for actual, expected in zip(
      pose.translation, (0.15, 0.15, 1.05), strict=True
    ):
      self.assertAlmostEqual(float(actual), expected, places=5)

  def test_writes_no_grasp_frames(self) -> None:
    """Grasp planning belongs to the grasp planner, not perception."""
    world, _ = _world_with_stock()
    update_detected_workpiece_pose(
      mock.Mock(object_world=world), _make_params(pos_z=1.0)
    )
    world.create_frame.assert_not_called()
    world.update_transform.assert_called_once()

  def test_enforces_min_safe_z(self) -> None:
    world, _ = _world_with_stock()
    with self.assertRaisesRegex(ValueError, "below "):
      update_detected_workpiece_pose(
        mock.Mock(object_world=world), _make_params(pos_z=0.5, min_safe_z=0.95)
      )
    world.update_transform.assert_not_called()


class RandomPlacementTest(absltest.TestCase):
  def test_randomize_placement_frame_shifts_pregrasp_and_grasp_in_lockstep(
    self,
  ) -> None:
    """Verifies pre_grasp and grasp receive identical XY/RZ shifts."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_world.root = mock_root

    pregrasp_initial = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([1.0, 0.0, 0.0, 0.0])),
      [0.30, -0.20, 1.13],
    )
    grasp_initial = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([1.0, 0.0, 0.0, 0.0])),
      [0.30, -0.20, 1.05],
    )

    def _get_transform(node_a: Any, node_b: Any) -> data_types.Pose3:
      del node_a
      if node_b is mock_root.pre_grasp:
        return pregrasp_initial
      if node_b is mock_root.grasp:
        return grasp_initial
      raise AssertionError(f"Unexpected node_b: {node_b}")

    mock_world.get_transform.side_effect = _get_transform
    context = mock.MagicMock()
    context.object_world = mock_world

    params = _make_params(
      parent_object="root",
      frame_name="pre_grasp",
      return_center_x=0.4,
      return_center_y=0.0,
      return_bounds_x=0.02,
      return_bounds_y=0.04,
      return_bounds_rz_degrees=20.0,
      grasp_frame_name="grasp",
    )
    randomize_placement_frame(context, params)

    self.assertEqual(mock_world.update_transform.call_count, 2)
    updated_poses = {
      call.kwargs["node_b"]: call.kwargs["a_t_b"]
      for call in mock_world.update_transform.call_args_list
    }
    pregrasp_updated = updated_poses[mock_root.pre_grasp]
    grasp_updated = updated_poses[mock_root.grasp]

    self.assertAlmostEqual(
      float(pregrasp_updated.translation[0]),
      float(grasp_updated.translation[0]),
      places=6,
    )
    self.assertAlmostEqual(
      float(pregrasp_updated.translation[1]),
      float(grasp_updated.translation[1]),
      places=6,
    )
    self.assertGreaterEqual(float(pregrasp_updated.translation[0]), 0.39)
    self.assertLessEqual(float(pregrasp_updated.translation[0]), 0.41)
    self.assertGreaterEqual(float(pregrasp_updated.translation[1]), -0.02)
    self.assertLessEqual(float(pregrasp_updated.translation[1]), 0.02)
    self.assertAlmostEqual(
      float(pregrasp_updated.translation[2]), 1.13, places=6
    )
    self.assertAlmostEqual(float(grasp_updated.translation[2]), 1.05, places=6)

    q_pre = pregrasp_updated.rotation.quaternion
    q_grasp = grasp_updated.rotation.quaternion
    self.assertAlmostEqual(float(q_pre.x), float(q_grasp.x), places=6)
    self.assertAlmostEqual(float(q_pre.y), float(q_grasp.y), places=6)
    self.assertAlmostEqual(float(q_pre.z), float(q_grasp.z), places=6)
    self.assertAlmostEqual(float(q_pre.w), float(q_grasp.w), places=6)


class RegraspFramesTest(absltest.TestCase):
  def test_saves_the_part_pose_under_the_grasp_frame(self):
    for existing in (False, True):
      with self.subTest(existing=existing):
        world = mock.MagicMock()
        world.root.frame_names = ["grasp_workpiece"] if existing else []
        grasp = mock.Mock(name="grasp")
        saved = mock.Mock(name="grasp_workpiece")
        world.get_frame.side_effect = lambda name, _, g=grasp, w=saved: (
          g if name == "grasp" else w
        )
        params = _make_params(
          workpiece_object_name="bracket",
          grasp_frame_name="grasp",
          workpiece_frame_name="grasp_workpiece",
        )
        exec(  # noqa: S102
          load_python_script(save_workpiece_at_grasp),
          {"context": mock.Mock(object_world=world), "params": params},
        )
        world.get_object.assert_called_once_with("bracket")
        world.get_transform.assert_called_once_with(
          grasp, world.get_object.return_value
        )
        if existing:
          world.update_transform.assert_called_once_with(
            node_a=grasp,
            node_b=saved,
            a_t_b=world.get_transform.return_value,
          )
        else:
          world.create_frame.assert_called_once_with(
            frame_name="grasp_workpiece",
            parent=grasp,
            parent_t_frame=world.get_transform.return_value,
          )

  def test_places_the_part_at_the_target_despite_changed_grip_and_return_shift(
    self,
  ):
    params = _make_params(
      workpiece_object_name="bracket",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      workpiece_frame_name="grasp_workpiece",
      grip_grasp_frame_name="grasp_unload",
      grip_pregrasp_frame_name="pre_grasp_unload",
      return_grasp_frame_name="grasp_return",
      return_pregrasp_frame_name="pre_grasp_return",
    )
    # Noncommuting rotations catch swapped transforms and wrong-frame offsets.
    desired = data_types.Pose3.from_vec7(
      [0.25, -0.15, 0.72, 0, 0, math.sin(0.3), math.cos(0.3)]
    )
    part_t_tool = data_types.Pose3.from_vec7(
      [0.02, -0.01, 0.04, math.sin(0.5), 0, 0, math.cos(0.5)]
    )
    # The unload's pregrasp sits 0.1 m from its grasp, here not along tool Z.
    grip_t_pregrasp = data_types.Pose3(translation=[0.06, 0.0, -0.08])
    for existing in (False, True):
      with self.subTest(existing=existing):
        world = mock.MagicMock()
        world.root.frame_names = (
          ["grasp_return", "pre_grasp_return"] if existing else []
        )
        world.get_transform.side_effect = [
          desired,
          part_t_tool,
          grip_t_pregrasp,
        ]
        exec(  # noqa: S102
          load_python_script(update_regrasp_return_frames),
          {"context": mock.Mock(object_world=world), "params": params},
        )
        calls = (
          world.update_transform.call_args_list
          if existing
          else world.create_frame.call_args_list
        )
        self.assertLen(calls, 2)
        poses = [
          call.kwargs["a_t_b" if existing else "parent_t_frame"]
          for call in calls
        ]
        self.assertTrue(
          (poses[0] * part_t_tool.inverse()).almost_equal(desired)
        )
        self.assertTrue(
          (poses[0].inverse() * poses[1]).almost_equal(
            data_types.Pose3(translation=[0, 0, -0.1])
          )
        )
        self.assertEqual(
          world.get_transform.call_args_list[:2],
          [
            mock.call(world.root, world.get_frame.return_value),
            mock.call(
              world.get_object.return_value, world.get_frame.return_value
            ),
          ],
        )
        world.get_frame.assert_any_call("grasp_unload", "root")
        world.get_frame.assert_any_call("pre_grasp_unload", "root")


class MathUtilsTest(absltest.TestCase):
  """Unit tests for math_utils helper functions."""

  def test_normalize_angle_and_joint_angles(self) -> None:
    self.assertAlmostEqual(normalize_angle(0.0), 0.0)
    self.assertAlmostEqual(normalize_angle(1.5 * math.pi), -0.5 * math.pi)
    self.assertAlmostEqual(normalize_angle(-1.5 * math.pi), 0.5 * math.pi)

    wrapped = normalize_joint_angles([0.0, 2.5 * math.pi, -2.5 * math.pi])
    self.assertAlmostEqual(wrapped[0], 0.0)
    self.assertAlmostEqual(wrapped[1], 0.5 * math.pi)
    self.assertAlmostEqual(wrapped[2], -0.5 * math.pi)

  def test_normalize_and_describe_motion_types(self) -> None:
    self.assertEqual(normalize_motion_types("LINEAR", 3), ["LINEAR"] * 3)
    self.assertEqual(
      normalize_motion_types(["ANY", "LINEAR"], 2), ["ANY", "LINEAR"]
    )
    with self.assertRaisesRegex(ValueError, "motion_type has 2 entries"):
      normalize_motion_types(["ANY", "LINEAR"], 3)

    self.assertEqual(describe_motion_types(["ANY", "ANY"]), "ANY")
    self.assertEqual(describe_motion_types(["ANY", "LINEAR"]), "ANY/LINEAR")

  def test_object_exists_in_world(self) -> None:
    self.assertFalse(object_exists_in_world(None, "root"))
    self.assertFalse(object_exists_in_world(mock.MagicMock(), None))
    self.assertFalse(object_exists_in_world(mock.MagicMock(), ""))

    mock_solution = mock.MagicMock()
    mock_solution.world.list_object_names.return_value = ["root", "ur_module"]
    self.assertTrue(object_exists_in_world(mock_solution, "ur_module"))
    self.assertFalse(object_exists_in_world(mock_solution, "cnc_enclosure"))

  def test_create_transform_node_ref(self) -> None:
    obj_ref = create_transform_node_ref("ur_module")
    self.assertEqual(obj_ref.by_name.object.object_name, "ur_module")
    self.assertFalse(obj_ref.by_name.HasField("frame"))

    frame_ref = create_transform_node_ref("gripper", "tool_frame")
    self.assertEqual(frame_ref.by_name.frame.object_name, "gripper")
    self.assertEqual(frame_ref.by_name.frame.frame_name, "tool_frame")


if __name__ == "__main__":
  absltest.main()
