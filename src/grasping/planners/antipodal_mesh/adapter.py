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

"""Antipodal mesh grasp planner, simulation only.

Runs the `antipodal_mesh_plan_grasp` skill, which plans a top-down
parallel-jaw grasp on two opposing contacts of the workpiece's mesh and checks
it with the motion planner.
"""

import dataclasses
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.config import AppConfig
from src.grasping import gripper_model_pb2
from src.grasping.interface import (
  PARALLEL_JAW,
  Approach,
  GraspExecution,
  GraspPlannerInterface,
)


@dataclasses.dataclass(frozen=True)
class AntipodalMeshSettings:
  """The `grasp.antipodal_mesh` config section.

  Attributes:
      pregrasp_distance_m: Height of the pregrasp above the planned grasp in
        meters.
      max_approach_tilt_deg: Largest angle, in degrees, between a contact
        face's normal and the horizontal plane.
  """

  pregrasp_distance_m: float = 0.1
  max_approach_tilt_deg: float = 10.0

  def __post_init__(self) -> None:
    if self.pregrasp_distance_m <= 0.0:
      raise ValueError(
        f"antipodal_mesh.pregrasp_distance_m must be positive, got "
        f"{self.pregrasp_distance_m}."
      )
    if not 0.0 <= self.max_approach_tilt_deg < 90.0:
      raise ValueError(
        f"antipodal_mesh.max_approach_tilt_deg must be in [0, 90), got "
        f"{self.max_approach_tilt_deg}."
      )


class AntipodalMeshGraspPlanner(GraspPlannerInterface):
  """Plans grasps on the workpiece's mesh with `antipodal_mesh_plan_grasp`."""

  # The skill checks the open jaws against the part and plans exact contacts,
  # so the jaws open first and move straight onto the contacts. The unload
  # plans again on the part where it was placed.
  execution = GraspExecution(
    open_before_planning=True,
    approach=Approach.DIRECT,
    replan_at_unload=True,
  )
  simulation_only = True

  def __init__(
    self,
    solution: Any,
    settings: AntipodalMeshSettings,
    gripper: gripper_model_pb2.ParallelJawGripper,
    robot_object_name: str,
    tool_object_name: str,
    tool_frame_name: str,
  ):
    if not robot_object_name:
      raise ValueError(
        "The antipodal mesh grasp planner needs the robot object name."
      )
    self._skill = solution.skills.ai.intrinsic.antipodal_mesh_plan_grasp
    self._settings = settings
    self._gripper = gripper
    self._robot_object_name = robot_object_name
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name

  def build_plan_grasp_task(
    self,
    workpiece_object_name: str,
    parent_object: str = "root",
    grasp_frame_name: str = "grasp",
    pregrasp_frame_name: str = "pre_grasp",
    name: str | None = None,
  ) -> bt.Node:
    if parent_object != "root":
      raise ValueError("Mesh grasp frames must be parented to root.")
    return bt.Task(
      name=name or "Plan Antipodal Mesh Grasp",
      action=self._skill(
        workpiece_object=workpiece_object_name,
        robot_object=self._robot_object_name,
        tool_object=self._tool_object_name,
        tool_frame=self._tool_frame_name,
        grasp_frame=grasp_frame_name,
        pregrasp_frame=pregrasp_frame_name,
        pregrasp_distance_m=self._settings.pregrasp_distance_m,
        max_approach_tilt_deg=self._settings.max_approach_tilt_deg,
        gripper=self._gripper,
      ),
    )


def create(
  solution: Any, config: AppConfig, settings: AntipodalMeshSettings
) -> AntipodalMeshGraspPlanner:
  """Builds the planner from the cell config.

  Raises:
      ValueError: If the cell's `gripper.model` is not a parallel-jaw gripper.
  """
  model = config.gripper.model
  if model is None or model.WhichOneof("kind") != PARALLEL_JAW:
    raise ValueError(
      "The antipodal mesh grasp planner needs the jaw geometry in "
      "'gripper.model.parallel_jaw' of the cell config."
    )
  return AntipodalMeshGraspPlanner(
    solution,
    settings,
    model.parallel_jaw,
    robot_object_name=config.robot.arm_part_name,
    tool_object_name=config.robot.tool_object_name,
    tool_frame_name=config.robot.tool_frame_name,
  )
