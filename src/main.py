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

"""Main application entrypoint for the Open Machine Tending Solution."""

from collections.abc import Sequence
from dataclasses import replace
import os
from pathlib import Path

from absl import app, flags, logging
from intrinsic.solutions import deployments

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.config import AppConfig, load_app_config
from src.core.infeed import (
  InfeedMode,
  PerceptionInfeedStrategy,
)
from src.core.types import GraspPlannerType, SimulationMode
from src.hardware.grasp_planners import create_grasp_planner
from src.hardware.gripper import DioGripper, GripperInterface, RobotiqGripper
from src.hardware.machine import DioCncMachine
from src.hardware.robot import Robot
from src.hardware.vision import OrbbecVision
from src.utils.execution_utils import to_executive_simulation_mode
from tools.pose_estimation.cycle_overlay import CyclePoseOverlay

_CONFIG = flags.DEFINE_string(
  "config",
  "configs/omts/app_config.yaml",
  "Path to the cell YAML configuration file.",
)
_ADDRESS = flags.DEFINE_string(
  "address",
  "localhost:17080",
  "gRPC address of the running SBL solution deployment.",
)
_SIMULATION_MODE = flags.DEFINE_enum_class(
  "simulation_mode",
  None,
  SimulationMode,
  "Executive execution mode override: 'reality', 'preview', or 'fast_preview'.",
)
_NUM_CYCLES = flags.DEFINE_integer(
  "num_cycles",
  None,
  "Optional override for number of cycles (1 = single, >1 = finite Loop, <=0 = continuous Loop).",
)
_GRASP_PLANNER = flags.DEFINE_enum_class(
  "grasp_planner",
  None,
  GraspPlannerType,
  "Optional override for the grasp planner backend, taking precedence over "
  "the 'grasp' section of the cell config.",
)

_MESH_CAD_JSON = flags.DEFINE_string(
  "mesh_cad_json",
  None,
  "Actual installed object-coordinate CAD JSON for experimental mesh planning.",
)
_MESH_SIMULATION_ONLY = flags.DEFINE_bool(
  "mesh_simulation_only",
  False,
  "Explicitly confirm the connected deployment is simulation-only for mesh planning.",
)

_PUBLISH_POSE_OVERLAY = flags.DEFINE_bool(
  "publish_pose_overlay", True, "Publish evaluation pose snapshots during cycles."
)
_OVERLAY_CAD_JSON = flags.DEFINE_string(
  "overlay_cad_json", None, "Exact installed object-coordinate CAD JSON."
)
_OVERLAY_ROS_MANIFEST = flags.DEFINE_string(
  "overlay_ros_manifest", None, "Optional native ROS Pixi manifest path."
)


def run_machine_tending_pipeline(
  solution_address: str,
  config: AppConfig,
  simulation_mode: SimulationMode | None = None,
  num_cycles_override: int | None = None,
  grasp_planner_override: GraspPlannerType | None = None,
  publish_pose_overlay: bool = True,
  overlay_cad_json: str | None = None,
  overlay_ros_manifest: str | None = None,
) -> None:
  """Connects to the solution deployment and executes the machine tending BT.

  Args:
      solution_address: gRPC endpoint of the running SBL solution deployment.
      config: Validated cell configuration dataclass.
      simulation_mode: Optional executive execution mode override.
      num_cycles_override: Optional cycle count override (1 = single cycle,
        >1 = finite bt.Loop, <=0 = continuous bt.Loop).
      grasp_planner_override: Optional grasp planner backend override, taking
        precedence over `config.grasp`.
  """
  logging.info(
    "Connecting to Intrinsic solution at %s (cell: %s)...",
    solution_address,
    config.cell_name,
  )
  solution = deployments.connect(address=solution_address)

  gripper: GripperInterface
  if config.gripper.type == "dio":
    gripper = DioGripper(
      solution=solution,
      config=config.gripper,
    )
  else:
    gripper = RobotiqGripper(
      solution=solution,
      config=config.gripper,
    )

  robot = Robot(
    solution=solution,
    config=config.robot,
  )
  machine = (
    DioCncMachine(
      solution=solution,
      config=config.machine,
    )
    if config.machine is not None
    else None
  )
  overlay = None
  if publish_pose_overlay:
    cad_path = overlay_cad_json
    if cad_path is None and config.vision.pose_estimator_id == (
      "ai.intrinsic.raw_stock_2x3x5_estimator"
    ):
      cad_path = "/tmp/opencode/omts_estimator_cad.json"
    if cad_path and Path(cad_path).is_file():
      overlay = CyclePoseOverlay(
        solution,
        config.cycle.workpiece_id,
        config.vision.pose_estimator_id,
        cad_path,
        os.environ.get("BUILD_WORKSPACE_DIRECTORY", str(Path.cwd())),
        overlay_ros_manifest,
      )
    else:
      logging.warning("Cycle pose overlay disabled: exact CAD JSON unavailable.")
  vision = OrbbecVision(
    solution=solution,
    config=config.vision,
    observation_task_builder=overlay.build_task if overlay else None,
  )

  infeed_mode = InfeedMode(config.vision.infeed_mode)
  if infeed_mode != InfeedMode.PERCEPTION:
    raise ValueError(
      f"Unsupported infeed_mode '{config.vision.infeed_mode}'. Only "
      f"'{InfeedMode.PERCEPTION.value}' is currently supported."
    )
  infeed_strategy = PerceptionInfeedStrategy(
    config=config.vision,
    view_frame_name=config.frames.view_frame,
  )

  planner_type = (
    grasp_planner_override
    if grasp_planner_override is not None
    else config.grasp.planner_type
  )
  grasp_planner = create_grasp_planner(
    planner_type=planner_type,
    solution=solution,
    config=config.grasp,
    tool_object_name=config.robot.tool_object_name,
    tool_frame_name=config.robot.tool_frame_name,
  )

  tree = build_machine_tending_behavior_tree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    vision=vision,
    infeed_strategy=infeed_strategy,
    config=config,
    num_cycles_override=num_cycles_override,
    grasp_planner=grasp_planner,
  )

  exec_sim_mode = to_executive_simulation_mode(simulation_mode)
  num_cycles = (
    num_cycles_override
    if num_cycles_override is not None
    else config.cycle.num_cycles
  )
  logging.info(
    "Executing machine tending Behavior Tree (cycles=%d, grasp_planner=%s)...",
    num_cycles,
    planner_type.value,
  )
  if overlay:
    overlay.run(tree, simulation_mode=exec_sim_mode)
  else:
    solution.executive.run(tree, simulation_mode=exec_sim_mode)


def main(argv: Sequence[str]) -> None:
  """Parses CLI flags and launches the OMTS machine tending pipeline.

  Args:
      argv: Non-flag command-line arguments passed by `absl.app`.

  Raises:
      app.UsageError: If unexpected positional arguments are provided.
  """
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")
  config = load_app_config(_CONFIG.value)
  if _GRASP_PLANNER.value is GraspPlannerType.MESH:
    config = replace(
      config,
      grasp=replace(
        config.grasp,
        planner="mesh",
        mesh_cad_json=_MESH_CAD_JSON.value or config.grasp.mesh_cad_json,
        mesh_simulation_only=_MESH_SIMULATION_ONLY.value,
      ),
    )
  run_machine_tending_pipeline(
    solution_address=_ADDRESS.value,
    config=config,
    simulation_mode=_SIMULATION_MODE.value,
    num_cycles_override=_NUM_CYCLES.value,
    grasp_planner_override=_GRASP_PLANNER.value,
    publish_pose_overlay=_PUBLISH_POSE_OVERLAY.value,
    overlay_cad_json=_OVERLAY_CAD_JSON.value,
    overlay_ros_manifest=_OVERLAY_ROS_MANIFEST.value,
  )


if __name__ == "__main__":
  app.run(main)
