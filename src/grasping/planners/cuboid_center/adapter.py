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

"""Built-in cuboid-center grasp planner, the default.

Grasps the center of the workpiece's pose from above, across its longest
horizontal axis, and seats the gripper with a force-controlled touchdown. It
assumes a cuboid part such as `raw_stock_2x3x5`.
"""

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.config import AppConfig
from src.grasping.interface import GraspPlannerInterface, NoSettings
from src.grasping.planners.cuboid_center import algorithm
from src.utils.script_utils import create_python_script


class CuboidCenterGraspPlanner(GraspPlannerInterface):
  """Writes a top-down grasp at the center of the workpiece's pose."""

  def __init__(
    self,
    solution: Any,
    tool_object_name: str,
    tool_frame_name: str,
    approach_offset_z: float,
  ):
    """Initializes the planner.

    Args:
        solution: Connected solution, used to build the script signature.
        tool_object_name: Object owning the tool frame.
        tool_frame_name: Tool frame whose current orientation picks the closest
          of the four top-down grasp orientations.
        approach_offset_z: Height of the pregrasp above the grasp in meters.
    """
    self._solution = solution
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name
    self._approach_offset_z = approach_offset_z

  def build_plan_grasp_task(
    self,
    workpiece_object_name: str,
    parent_object: str = "root",
    grasp_frame_name: str = "grasp",
    pregrasp_frame_name: str = "pre_grasp",
    name: str | None = None,
  ) -> bt.Node:
    return bt.Task(
      name=name or "Plan Cuboid-Center Grasp",
      action=create_python_script(
        self._solution,
        algorithm.plan_cuboid_center_grasp,
        [
          ("string", "parent_object", parent_object),
          ("string", "workpiece_object_name", workpiece_object_name),
          ("string", "tool_object_name", self._tool_object_name),
          ("string", "tool_frame_name", self._tool_frame_name),
          ("float", "approach_offset_z", self._approach_offset_z),
          ("string", "grasp_frame_name", grasp_frame_name),
          ("string", "pregrasp_frame_name", pregrasp_frame_name),
        ],
      ),
    )


def create(
  solution: Any, config: AppConfig, settings: NoSettings
) -> CuboidCenterGraspPlanner:
  """Builds the planner from the cell config."""
  del settings
  return CuboidCenterGraspPlanner(
    solution,
    tool_object_name=config.robot.tool_object_name,
    tool_frame_name=config.robot.tool_frame_name,
    approach_offset_z=config.cycle.approach_offset_z,
  )
