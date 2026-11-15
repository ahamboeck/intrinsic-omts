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

"""Unit tests for strict cell YAML configuration loading."""

import dataclasses
import pathlib
import tempfile
from unittest import mock

import yaml
from absl import app
from absl.testing import absltest
from intrinsic.solutions import execution

from src import main as omts_main
from src.core.config import (
  AppConfig,
  GraspConfig,
  GripperConfig,
  load_app_config,
  with_workpiece,
)
from src.core.types import SimulationMode
from src.grasping import registry
from src.grasping.interface import GraspPlannerInterface
from src.grasping.planners.cuboid_center.adapter import (
  CuboidCenterGraspPlanner,
)
from src.grasping.planners.moveit.adapter import MoveItSettings


def _write_config(raw: dict) -> str:
  with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
    yaml.safe_dump(raw, tmp)
    return tmp.name


def _raw_config(path: str = "configs/omts/app_config.yaml") -> dict:
  return yaml.safe_load(pathlib.Path(path).read_text(encoding="utf-8"))


class ConfigTest(absltest.TestCase):
  def test_load_omts_app_config(self):
    config = load_app_config("configs/omts/app_config.yaml")
    self.assertIsInstance(config, AppConfig)
    self.assertEqual(config.cell_name, "omts")
    self.assertEqual(config.robot.arm_part_name, "ur_module")
    self.assertEqual(config.robot.enclosure_object_name, "enclosure")
    self.assertEqual(config.gripper.type, "robotiq")
    self.assertEqual(
      config.gripper.joint_name, "robotiq_hande_left_finger_joint"
    )
    self.assertIsNone(config.gripper.action_name)
    self.assertIsNone(config.gripper.dio_open_pin)
    self.assertIsNone(config.gripper.dio_close_pin)
    self.assertIsNone(config.gripper.dio_device_name)
    self.assertIsNone(config.gripper.dio_output_block_name)
    self.assertEqual(config.machine.enclosure_object_name, "cnc_enclosure")
    self.assertEqual(config.machine.vise_object_name, "schunk_egp_64nnb")
    self.assertEqual(config.machine.output_block_name, "standard_out")
    self.assertEqual(config.machine.input_block_name, "standard_in")
    self.assertEqual(config.frames.transit_frame, "transit")
    self.assertEqual(config.frames.preplace_vise_frame, "vise_pre_place")
    self.assertEqual(config.frames.place_vise_frame, "vise_place")
    self.assertEqual(
      config.vision.perception_service_name, "nvidia_pose_estimator_service"
    )
    self.assertEqual(
      config.vision.pose_estimator_id,
      "ai.intrinsic.raw_stock_2x3x5_estimator_text",
    )
    self.assertEqual(config.vision.sensor_ids, (1, 4))
    self.assertEqual(config.cycle.workpiece_id, "raw_stock_2x3x5")

  def test_load_lab_bb_01_app_config(self):
    config = load_app_config("configs/lab_bb_01/app_config.yaml")
    self.assertIsInstance(config, AppConfig)
    self.assertEqual(config.cell_name, "lab_bb_01")
    self.assertEqual(config.gripper.type, "robotiq")
    self.assertIsNone(config.gripper.dio_output_block_name)
    self.assertIsNone(config.machine)
    self.assertIsNone(config.frames.transit_frame)
    self.assertEqual(config.robot.enclosure_object_name, "enclosure")

  def test_omitted_grasp_section_defaults_to_cuboid_center(self):
    raw = _raw_config()
    del raw["grasp"]

    config = load_app_config(_write_config(raw))
    self.assertEqual(config.grasp.planner, "cuboid_center")
    self.assertFalse(config.grasp.simulation_only)
    self.assertEqual(config.grasp.settings, {})

  def test_grasp_section_selects_moveit_and_keeps_its_section(self):
    raw = _raw_config()
    raw["grasp"]["planner"] = "moveit"
    raw["grasp"]["moveit"]["surfaces"] = [4]

    config = load_app_config(_write_config(raw))
    self.assertEqual(config.grasp.planner, "moveit")
    self.assertEqual(
      registry.planner_settings(config.grasp, "moveit").surfaces, (4,)
    )

  def test_grasp_section_reads_the_simulation_only_acknowledgement(self):
    raw = _raw_config()
    raw["grasp"] = {"planner": "antipodal_mesh", "simulation_only": True}

    config = load_app_config(_write_config(raw))
    self.assertTrue(config.grasp.simulation_only)
    self.assertEqual(config.grasp.settings, {})

  def test_planner_section_must_be_a_mapping(self):
    raw = _raw_config()
    raw["grasp"]["moveit"] = 4
    with self.assertRaisesRegex(ValueError, "'grasp.moveit'"):
      load_app_config(_write_config(raw))

  def test_loads_the_gripper_model(self):
    config = load_app_config("configs/omts/app_config.yaml")
    self.assertEqual(config.gripper.model.WhichOneof("kind"), "parallel_jaw")
    self.assertAlmostEqual(config.gripper.model.parallel_jaw.max_width_m, 0.06)

  def test_gripper_model_is_optional(self):
    raw = _raw_config()
    del raw["gripper"]["model"]
    self.assertIsNone(load_app_config(_write_config(raw)).gripper.model)

  def test_invalid_gripper_model_fails_loudly(self):
    raw = _raw_config()
    raw["gripper"]["model"] = {"parallel_jaw": {"max_width": 0.06}}
    with self.assertRaisesRegex(ValueError, "gripper.model"):
      load_app_config(_write_config(raw))
    raw["gripper"]["model"] = {}
    with self.assertRaisesRegex(ValueError, "gripper kind"):
      load_app_config(_write_config(raw))

  def test_missing_required_section_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
  tool_frame_name: "tool_frame"
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration section 'gripper'"
    ):
      load_app_config(tmp_path)

  def test_missing_required_field_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*tool_frame_name"
    ):
      load_app_config(tmp_path)

  def test_missing_robotiq_gripper_field_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
  tool_frame_name: "tool_frame"
gripper:
  type: "robotiq"
  joint_name: "robotiq_hande_left_finger_joint"
  open_position: 0.025
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*close_position"
    ):
      load_app_config(tmp_path)

  def test_missing_dio_gripper_fields_and_invalid_yaml_root(self):
    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*dio_output_block_name"
    ):
      GripperConfig(type="dio", dio_open_pin=0, dio_close_pin=1)

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write("- not_a_mapping\n")
      list_path = tmp.name
    with self.assertRaisesRegex(ValueError, "must contain a top-level mapping"):
      load_app_config(list_path)

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write("robot:\n  arm_part_name: ur_module\n")
      no_cell_path = tmp.name
    with self.assertRaisesRegex(KeyError, "Missing required field 'cell_name'"):
      load_app_config(no_cell_path)

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_run_machine_tending_pipeline_omts_and_dio_and_errors(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    mock_solution = mock.MagicMock()
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "nvidia_pose_estimator_service": mock.MagicMock(),
    }
    mock_connect.return_value = mock_solution
    mock_tree = mock.MagicMock()
    mock_build_bt.return_value = mock_tree

    omts_cfg = load_app_config("configs/omts/app_config.yaml")
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=omts_cfg,
      simulation_mode=SimulationMode.PREVIEW,
      num_cycles_override=2,
    )
    mock_solution.executive.run.assert_called_once_with(
      mock_tree,
      simulation_mode=execution.Executive.SimulationMode.PREVIEW,
    )

    # Test with DIO gripper and lab_bb_01 (no CNC machine)
    lab_cfg = load_app_config("configs/lab_bb_01/app_config.yaml")
    dio_cfg = dataclasses.replace(
      lab_cfg,
      gripper=GripperConfig(
        type="dio",
        dio_open_pin=0,
        dio_close_pin=1,
        dio_device_name="ur_module",
        dio_output_block_name="standard_out",
      ),
    )
    mock_solution.executive.run.reset_mock()
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=dio_cfg,
    )
    self.assertIsNone(mock_build_bt.call_args.kwargs["machine"])
    mock_solution.executive.run.assert_called_once_with(
      mock_tree, simulation_mode=None
    )

    # Unsupported infeed_mode raises ValueError
    grid_cfg = dataclasses.replace(
      lab_cfg,
      vision=dataclasses.replace(lab_cfg.vision, infeed_mode="grid"),
    )
    with self.assertRaisesRegex(ValueError, "Unsupported infeed_mode 'grid'"):
      omts_main.run_machine_tending_pipeline(
        solution_address="localhost:17080",
        config=grid_cfg,
      )

    # Extra CLI positional argument raises UsageError
    with self.assertRaises(app.UsageError):
      omts_main.main(["main.py", "unexpected_arg"])


class _FakeGraspPlanner(GraspPlannerInterface):
  def __init__(self, settings):
    self.settings = settings

  def build_plan_grasp_task(self, workpiece_object_name, *args, **kwargs):
    raise NotImplementedError


@dataclasses.dataclass(frozen=True)
class _FakeSettings:
  cup_count: int = 1


class GraspPlannerRegistryTest(absltest.TestCase):
  """Unit tests for `src.grasping.registry`."""

  def setUp(self):
    super().setUp()
    self.config = load_app_config("configs/omts/app_config.yaml")

  def _with_grasp(self, **changes) -> AppConfig:
    return dataclasses.replace(
      self.config, grasp=dataclasses.replace(self.config.grasp, **changes)
    )

  def test_rejects_an_unknown_planner(self):
    with self.assertRaisesRegex(
      ValueError, "Unsupported grasp planner 'nonexistent_planner'"
    ) as ctx:
      registry.validate(GraspConfig(planner="nonexistent_planner"))
    self.assertIn("cuboid_center", str(ctx.exception))

  def test_rejects_a_section_for_an_unknown_planner(self):
    with self.assertRaisesRegex(ValueError, "nonexistent_planner"):
      registry.validate(GraspConfig(settings={"nonexistent_planner": {}}))

  def test_rejects_unknown_fields_in_a_planner_section(self):
    grasp = GraspConfig(settings={"moveit": {"surface": [4]}})
    with self.assertRaisesRegex(ValueError, r"\['surface'\].*grasp.moveit"):
      registry.validate(grasp)

  def test_omitted_section_uses_defaults(self):
    self.assertEqual(
      registry.planner_settings(GraspConfig(), "moveit"), MoveItSettings()
    )

  def test_moveit_settings_are_validated(self):
    self.assertEqual(MoveItSettings().surfaces, ())
    with self.assertRaisesRegex(ValueError, "moveit.surfaces"):
      MoveItSettings(surfaces=(0, 6))
    with self.assertRaisesRegex(ValueError, "moveit.num_rotations"):
      MoveItSettings(num_rotations=0)
    with self.assertRaisesRegex(ValueError, "moveit.retract_dist_m"):
      MoveItSettings(retract_dist_m=0.0)
    with self.assertRaisesRegex(ValueError, "moveit.timeout_ms"):
      MoveItSettings(timeout_ms=-1.0)
    with self.assertRaisesRegex(ValueError, "moveit.settle_seconds"):
      MoveItSettings(settle_seconds=-0.1)
    self.assertEqual(MoveItSettings(settle_seconds=0.0).settle_seconds, 0.0)

  def test_cuboid_center_is_a_planner(self):
    planner = registry.create_grasp_planner(mock.MagicMock(), self.config)
    self.assertIsInstance(planner, CuboidCenterGraspPlanner)

  def test_moveit_forwards_its_section(self):
    planner = registry.create_grasp_planner(
      mock.MagicMock(), self.config, "moveit"
    )

    self.assertIsInstance(planner, GraspPlannerInterface)
    self.assertEqual(planner.default_surfaces, (0, 1, 4, 5))
    self.assertEqual(planner.default_num_rotations, 4)
    self.assertEqual(planner.default_settle_seconds, 0.3)

  def test_moveit_without_its_skill_fails_loudly(self):
    """Fails at startup, not mid-cycle, if the solution lacks the skill."""
    solution = mock.MagicMock()
    del solution.skills.ai.intrinsic.moveit_plan_grasp_skill

    with self.assertRaisesRegex(ValueError, "moveit_plan_grasp_skill"):
      registry.create_grasp_planner(solution, self.config, "moveit")

  def test_simulation_only_planner_needs_the_acknowledgement(self):
    # As when --grasp_planner=antipodal_mesh overrides a cuboid_center config.
    with self.assertRaisesRegex(ValueError, "simulation only"):
      registry.create_grasp_planner(
        mock.MagicMock(), self.config, "antipodal_mesh"
      )
    planner = registry.create_grasp_planner(
      mock.MagicMock(), self._with_grasp(simulation_only=True), "antipodal_mesh"
    )
    self.assertTrue(planner.simulation_only)

  def test_rejects_a_gripper_kind_the_planner_does_not_support(self):
    planner = _FakeGraspPlanner(None)
    planner.gripper_kinds = frozenset({"suction"})
    entry = registry.PlannerEntry(lambda *_: planner)
    with mock.patch.dict(registry.PLANNERS, {"fake": entry}):
      with self.assertRaisesRegex(ValueError, "'parallel_jaw' gripper"):
        registry.create_grasp_planner(mock.MagicMock(), self.config, "fake")

  def test_a_new_planner_only_needs_a_registry_entry(self):
    """Registering a planner makes it selectable and configurable."""
    entry = registry.PlannerEntry(
      lambda solution, config, settings: _FakeGraspPlanner(settings),
      _FakeSettings,
    )
    config = self._with_grasp(
      planner="fake", settings={"fake": {"cup_count": 2}}
    )
    with mock.patch.dict(registry.PLANNERS, {"fake": entry}):
      planner = registry.create_grasp_planner(mock.MagicMock(), config)
    self.assertEqual(planner.settings, _FakeSettings(cup_count=2))


class WithWorkpieceTest(absltest.TestCase):
  def test_names_the_workpiece_and_its_estimator(self):
    lab = load_app_config("configs/lab_bb_01/app_config.yaml")
    banana = with_workpiece(lab, "banana")
    self.assertEqual(banana.cycle.workpiece_id, "banana")
    self.assertEqual(banana.vision.scene_object_id, "ai.intrinsic.banana")
    self.assertEqual(
      banana.vision.pose_estimator_id, "ai.intrinsic.banana_estimator_text"
    )
    self.assertEqual(banana.frames, lab.frames)

  def test_shipped_raw_stock_follows_the_naming(self):
    lab = load_app_config("configs/lab_bb_01/app_config.yaml")
    self.assertEqual(with_workpiece(lab, "raw_stock_2x3x5"), lab)


class GraspPlannerSelectionTest(absltest.TestCase):
  """Tests how the entry point resolves which grasp planner to use."""

  def _run_pipeline(self, mock_build_bt, mock_connect, **kwargs):
    mock_solution = mock.MagicMock()
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "nvidia_pose_estimator_service": mock.MagicMock(),
    }
    mock_connect.return_value = mock_solution
    mock_build_bt.return_value = mock.MagicMock()
    config = load_app_config("configs/omts/app_config.yaml")
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=config,
      **kwargs,
    )
    return mock_build_bt.call_args.kwargs

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_cuboid_center_is_passed_to_the_tree(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    kwargs = self._run_pipeline(mock_build_bt, mock_connect)
    self.assertIsInstance(kwargs["grasp_planner"], CuboidCenterGraspPlanner)

  @mock.patch("src.main.registry.create_grasp_planner")
  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_cli_override_takes_precedence_over_config(
    self,
    mock_build_bt: mock.MagicMock,
    mock_connect: mock.MagicMock,
    mock_factory: mock.MagicMock,
  ):
    """The shipped config says cuboid_center, so moveit can only be the flag."""
    planner = mock.MagicMock()
    mock_factory.return_value = planner

    kwargs = self._run_pipeline(
      mock_build_bt, mock_connect, grasp_planner_override="moveit"
    )

    self.assertEqual(mock_factory.call_args.args[2], "moveit")
    self.assertIs(kwargs["grasp_planner"], planner)

  @mock.patch("src.main.registry.create_grasp_planner")
  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_config_is_used_when_no_override_is_given(
    self,
    mock_build_bt: mock.MagicMock,
    mock_connect: mock.MagicMock,
    mock_factory: mock.MagicMock,
  ):
    self._run_pipeline(mock_build_bt, mock_connect)

    self.assertEqual(mock_factory.call_args.args[2], "cuboid_center")

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_invalid_planner_is_rejected_before_connecting(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    with self.assertRaisesRegex(ValueError, "Unsupported grasp planner"):
      self._run_pipeline(
        mock_build_bt, mock_connect, grasp_planner_override="nonexistent"
      )
    mock_connect.assert_not_called()


if __name__ == "__main__":
  absltest.main()
