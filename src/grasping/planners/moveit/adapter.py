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

"""Registers the `intrinsic-moveit` grasp planner, configured by `grasp.moveit`.

The planner itself lives in `third_party/intrinsic_moveit`.
"""

import dataclasses
from typing import Any

from src.core.config import AppConfig
from third_party.intrinsic_moveit.moveit_grasp_planner import MoveItGraspPlanner


@dataclasses.dataclass(frozen=True)
class MoveItSettings:
  """The `grasp.moveit` config section.

  Attributes:
      tool_frame_name: MoveIt robot model link used as the grasp TCP.
      group_name: MoveIt planning group for the arm.
      end_effector_group: MoveIt end-effector group.
      surfaces: Workpiece surfaces to sample grasp candidates on (0:+X, 1:-X,
        2:+Y, 3:-Y, 4:+Z, 5:-Z). Empty samples every surface.
      num_rotations: Grasp candidates per surface, spread evenly about its
        normal.
      retract_dist_m: Distance in meters between the planned grasp and
        pre-grasp.
      timeout_ms: Grasp planning timeout in milliseconds.
      settle_seconds: Dwell before planning, letting the MoveIt planning scene
        catch up with the workpiece pose that perception just published.
  """

  tool_frame_name: str = "hande_tcp"
  group_name: str = "ur_manipulator"
  end_effector_group: str = "hand"
  surfaces: tuple[int, ...] = ()
  num_rotations: int = 4
  retract_dist_m: float = 0.05
  timeout_ms: float = 15000.0
  settle_seconds: float = 0.3

  def __post_init__(self) -> None:
    object.__setattr__(self, "surfaces", tuple(self.surfaces))
    if any(not 0 <= surface <= 5 for surface in self.surfaces):
      raise ValueError(
        f"moveit.surfaces entries must be in 0..5, got {list(self.surfaces)}."
      )
    if self.num_rotations <= 0:
      raise ValueError(
        f"moveit.num_rotations must be positive, got {self.num_rotations}."
      )
    if self.retract_dist_m <= 0.0:
      raise ValueError(
        f"moveit.retract_dist_m must be positive, got {self.retract_dist_m}."
      )
    if self.timeout_ms <= 0.0:
      raise ValueError(
        f"moveit.timeout_ms must be positive, got {self.timeout_ms}."
      )
    if self.settle_seconds < 0.0:
      raise ValueError(
        f"moveit.settle_seconds must not be negative, got "
        f"{self.settle_seconds}."
      )


def create(
  solution: Any, config: AppConfig, settings: MoveItSettings
) -> MoveItGraspPlanner:
  """Builds the planner from the cell config.

  Raises:
      ValueError: If the MoveIt grasp skill is missing from the solution.
  """
  return MoveItGraspPlanner(
    solution=solution,
    tool_frame_name=settings.tool_frame_name,
    tool_object_name=config.robot.tool_object_name,
    group_name=settings.group_name,
    end_effector_group=settings.end_effector_group,
    retract_dist_m=settings.retract_dist_m,
    timeout_ms=settings.timeout_ms,
    surfaces=settings.surfaces,
    num_rotations=settings.num_rotations,
    settle_seconds=settings.settle_seconds,
  )
