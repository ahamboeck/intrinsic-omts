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

"""Suction mesh grasp planner, simulation only.

Runs the `suction_mesh_plan_grasp` skill, which places a suction cup on a
level, flat patch of the workpiece's mesh and checks the approach with the
motion planner.
"""

import dataclasses
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.config import AppConfig
from src.grasping import gripper_model_pb2
from src.grasping.interface import (
  SUCTION,
  Approach,
  GraspExecution,
  GraspPlannerInterface,
)


@dataclasses.dataclass(frozen=True)
class SuctionMeshSettings:
  """The `grasp.suction_mesh` config section.

  Attributes:
      pregrasp_distance_m: Height of the pregrasp above the planned grasp in
        meters.
      max_approach_tilt_deg: Largest angle, in degrees, between a face under
        the cup and the horizontal plane.
      flatness_tolerance_m: Largest height difference of the surface under the
        cup, in meters, that the cup still seals on.
      press_depth_m: How far the grasp presses the cup below the surface, in
        meters, so the cup touches the part.
  """

  pregrasp_distance_m: float = 0.1
  max_approach_tilt_deg: float = 15.0
  flatness_tolerance_m: float = 0.002
  press_depth_m: float = 0.001

  def __post_init__(self) -> None:
    if self.pregrasp_distance_m <= 0.0:
      raise ValueError(
        f"suction_mesh.pregrasp_distance_m must be positive, got "
        f"{self.pregrasp_distance_m}."
      )
    if not 0.0 <= self.max_approach_tilt_deg < 90.0:
      raise ValueError(
        f"suction_mesh.max_approach_tilt_deg must be in [0, 90), got "
        f"{self.max_approach_tilt_deg}."
      )
    if self.flatness_tolerance_m < 0.0:
      raise ValueError(
        f"suction_mesh.flatness_tolerance_m must not be negative, got "
        f"{self.flatness_tolerance_m}."
      )
    if not 0.0 <= self.press_depth_m < 0.01:
      raise ValueError(
        f"suction_mesh.press_depth_m must be in [0, 0.01), got "
        f"{self.press_depth_m}."
      )


class SuctionMeshGraspPlanner(GraspPlannerInterface):
  """Plans suction grasps on the workpiece's mesh with `suction_mesh_plan_grasp`."""

  # The cup is placed on an exact spot and must touch it, so the tool moves
  # straight onto the grasp, which presses the cup slightly into the surface.
  # Suction holds wherever the cup lands, so the unload plans again on the
  # part where it was placed.
  execution = GraspExecution(
    open_before_planning=False,
    approach=Approach.DIRECT,
    replan_at_unload=True,
  )
  gripper_kinds = frozenset({SUCTION})
  simulation_only = True

  def __init__(
    self,
    solution: Any,
    settings: SuctionMeshSettings,
    gripper: gripper_model_pb2.SuctionGripper,
    robot_object_name: str,
    tool_object_name: str,
    tool_frame_name: str,
  ):
    if not robot_object_name:
      raise ValueError(
        "The suction mesh grasp planner needs the robot object name."
      )
    self._skill = solution.skills.ai.intrinsic.suction_mesh_plan_grasp
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
      name=name or "Plan Suction Mesh Grasp",
      action=self._skill(
        workpiece_object=workpiece_object_name,
        robot_object=self._robot_object_name,
        tool_object=self._tool_object_name,
        tool_frame=self._tool_frame_name,
        grasp_frame=grasp_frame_name,
        pregrasp_frame=pregrasp_frame_name,
        pregrasp_distance_m=self._settings.pregrasp_distance_m,
        max_approach_tilt_deg=self._settings.max_approach_tilt_deg,
        flatness_tolerance_m=self._settings.flatness_tolerance_m,
        press_depth_m=self._settings.press_depth_m,
        gripper=self._gripper,
      ),
    )


def create(
  solution: Any, config: AppConfig, settings: SuctionMeshSettings
) -> SuctionMeshGraspPlanner:
  """Builds the planner from the cell config.

  Raises:
      ValueError: If the cell's `gripper.model` is not a suction gripper.
  """
  model = config.gripper.model
  if model is None or model.WhichOneof("kind") != SUCTION:
    raise ValueError(
      "The suction mesh grasp planner needs the cup geometry in "
      "'gripper.model.suction' of the cell config."
    )
  return SuctionMeshGraspPlanner(
    solution,
    settings,
    model.suction,
    robot_object_name=config.robot.arm_part_name,
    tool_object_name=config.robot.tool_object_name,
    tool_frame_name=config.robot.tool_frame_name,
  )
