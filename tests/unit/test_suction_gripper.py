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

"""Tests for the suction gripper: its config, adapter and command skill."""

import types
from unittest import mock

import grpc
from absl.testing import absltest
from intrinsic.hardware.gripper.eoat import eoat_service_pb2
from intrinsic.skills.python import skill_interface

from src.core.config import GripperConfig, load_app_config
from src.grasping import registry
from src.hardware.gripper import SuctionGripper
from src.skills.control_suction_gripper import (
  control_suction_gripper,
  control_suction_gripper_pb2,
)

_SUCTION_CONFIG = "configs/lab_bb_01/app_config_suction.yaml"


class SuctionConfigTest(absltest.TestCase):
  def test_loads_the_suction_cell(self):
    config = load_app_config(_SUCTION_CONFIG)
    self.assertEqual(config.gripper.type, "suction")
    self.assertEqual(config.gripper.resource_name, "gripper")
    self.assertEqual(config.gripper.model.WhichOneof("kind"), "suction")
    self.assertAlmostEqual(config.gripper.model.suction.cup_diameter_m, 0.02)
    self.assertEqual(config.grasp.planner, "suction_mesh")
    self.assertTrue(config.grasp.simulation_only)

  def test_suction_needs_its_resource(self):
    with self.assertRaisesRegex(KeyError, "resource_name"):
      GripperConfig(type="suction")

  def test_parallel_jaw_planners_reject_the_suction_cell(self):
    config = load_app_config(_SUCTION_CONFIG)
    for planner in ("cuboid_center", "antipodal_mesh"):
      with self.subTest(planner=planner):
        with self.assertRaisesRegex(ValueError, "suction|parallel_jaw"):
          registry.create_grasp_planner(mock.MagicMock(), config, planner)

  def test_suction_planner_is_built_for_the_suction_cell(self):
    planner = registry.create_grasp_planner(
      mock.MagicMock(), load_app_config(_SUCTION_CONFIG)
    )
    self.assertEqual(type(planner).__name__, "SuctionMeshGraspPlanner")


class SuctionGripperAdapterTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.solution = mock.MagicMock()
    self.solution.resources = {"gripper": mock.sentinel.gripper_resource}
    self.skill = self.solution.skills.ai.intrinsic.control_suction_gripper
    self.skill.return_value = mock.MagicMock()
    self.gripper = SuctionGripper(
      self.solution, load_app_config(_SUCTION_CONFIG).gripper
    )

  @mock.patch("src.hardware.gripper.bt.Task")
  def test_close_turns_the_vacuum_on(self, task):
    self.gripper.build_close_task(name="Grip")

    kwargs = self.skill.call_args.kwargs
    self.assertIs(kwargs["suction_gripper"], mock.sentinel.gripper_resource)
    self.assertIn("grasp", kwargs)
    self.assertNotIn("release", kwargs)
    self.assertEqual(task.call_args.kwargs["name"], "Grip")

  @mock.patch("src.hardware.gripper.create_dwell_task")
  @mock.patch("src.hardware.gripper.bt.Sequence")
  @mock.patch("src.hardware.gripper.bt.Task")
  def test_open_turns_the_vacuum_off_and_waits_before_moving(
    self, unused_task, sequence, dwell
  ):
    self.gripper.build_open_task(name="Let Go")

    kwargs = self.skill.call_args.kwargs
    self.assertIn("release", kwargs)
    self.assertNotIn("grasp", kwargs)
    self.assertEqual(dwell.call_args.args[0], 0.5)
    children = sequence.call_args.kwargs["children"]
    self.assertIs(children[-1], dwell.return_value)

  def test_missing_resource_fails_loudly(self):
    self.solution.resources = {}
    with self.assertRaisesRegex(ValueError, "'gripper' not found"):
      SuctionGripper(self.solution, load_app_config(_SUCTION_CONFIG).gripper)


class ControlSuctionGripperSkillTest(absltest.TestCase):
  def _execute(self, stub, **command):
    params = control_suction_gripper_pb2.SuctionGripperCommand(**command)
    with (
      mock.patch.object(
        control_suction_gripper.asset_utils, "connect"
      ) as connect,
      mock.patch.object(
        control_suction_gripper.eoat_service_pb2_grpc,
        "SuctionGripperStub",
        return_value=stub,
      ),
    ):
      control_suction_gripper.ControlSuctionGripper().execute(
        types.SimpleNamespace(params=params), None
      )
    return connect

  def test_grasp_turns_the_vacuum_on(self):
    stub = mock.Mock()
    connect = self._execute(stub, grasp=eoat_service_pb2.GraspRequest())

    stub.Grasp.assert_called_once()
    stub.Release.assert_not_called()
    self.assertEqual(
      connect.call_args.args[1], "grpc://intrinsic_proto.eoat.SuctionGripper"
    )

  def test_release_turns_the_vacuum_off(self):
    stub = mock.Mock()
    self._execute(stub, release=eoat_service_pb2.ReleaseRequest())
    stub.Release.assert_called_once()
    stub.Grasp.assert_not_called()

  def test_missing_command_is_rejected(self):
    with self.assertRaises(skill_interface.SkillError):
      self._execute(mock.Mock())

  def test_service_errors_become_skill_errors(self):
    class _Unavailable(grpc.RpcError):
      def details(self):
        return "connection refused"

    stub = mock.Mock()
    stub.Grasp.side_effect = _Unavailable()
    with self.assertRaisesRegex(skill_interface.SkillError, "refused"):
      self._execute(stub, grasp=eoat_service_pb2.GraspRequest())


if __name__ == "__main__":
  absltest.main()
