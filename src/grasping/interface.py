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

"""Contract between grasp planners and the pick, unload and return behaviours.

A planner writes the `grasp` and `pre_grasp` frames for a workpiece whose pose
is already in the world, or fails with a clear error. It also states, through
`GraspExecution`, how the behaviours should use its grasps. The behaviours own
the sequence; a planner only plans.
"""

import abc
import dataclasses
import enum
from typing import ClassVar

from intrinsic.solutions import behavior_tree as bt

PARALLEL_JAW = "parallel_jaw"
SUCTION = "suction"


class Approach(enum.Enum):
  """How the tool moves from the pregrasp onto the planned grasp.

  Attributes:
      TOUCHDOWN: Seats the tool with a force-controlled touchdown along tool +Z
        at the grasp frame and retracts slightly before closing. Suits grasps
        whose height is only approximate, such as the top of a detected part.
      DIRECT: Moves linearly to the grasp frame and closes there. Suits grasps
        planned on exact contacts.
  """

  TOUCHDOWN = "touchdown"
  DIRECT = "direct"


@dataclasses.dataclass(frozen=True)
class GraspExecution:
  """How the behaviours use a planner's grasps.

  Attributes:
      open_before_planning: Opens the gripper before planning, for planners
        that check the open gripper against the part. Otherwise the gripper
        opens after planning.
      approach: How the tool moves onto the grasp, at the pick and, when
        `replan_at_unload` is set, at the unload.
      replan_at_unload: Plans a new grasp on the part in the machine instead of
        touching down at the vise frame. The grip can then differ from the
        pick, so the return places the part from its saved infeed pose and the
        current grip.
  """

  open_before_planning: bool = False
  approach: Approach = Approach.TOUCHDOWN
  replan_at_unload: bool = False


@dataclasses.dataclass(frozen=True)
class NoSettings:
  """Settings of a planner that has none of its own."""


class GraspPlannerInterface(abc.ABC):
  """Writes grasp and pre-grasp frames for an already-localized workpiece.

  Attributes:
      execution: How the behaviours use this planner's grasps.
      gripper_kinds: Gripper kinds, the `kind` of `gripper.model` in the cell
        config, this planner can plan for.
      simulation_only: True if this planner has not been verified on hardware.
        Selecting it then requires the `grasp.simulation_only` acknowledgement.
  """

  execution: ClassVar[GraspExecution] = GraspExecution()
  gripper_kinds: ClassVar[frozenset[str]] = frozenset({PARALLEL_JAW})
  simulation_only: ClassVar[bool] = False

  @abc.abstractmethod
  def build_plan_grasp_task(
    self,
    workpiece_object_name: str,
    parent_object: str = "root",
    grasp_frame_name: str = "grasp",
    pregrasp_frame_name: str = "pre_grasp",
    name: str | None = None,
  ) -> bt.Node:
    """Builds the task that plans a grasp and writes it into the world.

    Implementations update the frames if they exist and may create them on
    `parent_object` otherwise, since the unload plans into frames that the
    scene does not declare.

    Args:
        workpiece_object_name: Object World name of the part to grasp.
        parent_object: Object owning the output frames.
        grasp_frame_name: Frame set to the planned grasp pose of the tool.
        pregrasp_frame_name: Frame set to the planned pre-grasp pose, which
          the behaviours approach first.
        name: Optional custom behavior tree task name.

    Returns:
        Behavior tree node that writes the planned frames when executed.
    """
    raise NotImplementedError
