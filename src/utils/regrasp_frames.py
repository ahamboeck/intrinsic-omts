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

"""Frames for returning a part whose grip changed at the unload.

Run as behavior tree `PythonScript`s, so these functions import only what the
script runtime provides. All poses are in SI units.
"""

from typing import Any


def _set_frame(world: Any, name: str, parent: Any, parent_t_frame: Any) -> None:
  """Updates frame `name` on root, or creates it under `parent`."""
  if name in world.root.frame_names:
    world.update_transform(
      node_a=parent, node_b=world.get_frame(name, "root"), a_t_b=parent_t_frame
    )
  else:
    world.create_frame(
      frame_name=name, parent=parent, parent_t_frame=parent_t_frame
    )


def save_workpiece_at_grasp(context: Any, params: Any) -> None:
  """Saves the part's pose relative to the planned infeed grasp.

  The saved frame is a child of the grasp frame, so it follows the grasp frame
  when the return shifts it.

  Args:
      context: `PythonScript` execution context providing `object_world`.
      params: Script parameters with `workpiece_object_name`,
        `grasp_frame_name` and `workpiece_frame_name`.
  """
  world = context.object_world
  grasp = world.get_frame(params.grasp_frame_name, "root")
  workpiece = world.get_object(params.workpiece_object_name)
  _set_frame(
    world,
    params.workpiece_frame_name,
    grasp,
    world.get_transform(grasp, workpiece),
  )


def update_regrasp_return_frames(context: Any, params: Any) -> None:
  """Solves the return tool targets from the saved part pose and current grip.

  The grasp target is `world_T_tool = world_T_saved_part * part_T_tool`, using
  the current grip. The pregrasp target sits as far back along tool -Z as the
  current grip's planned pregrasp sat from its grasp.

  Args:
      context: `PythonScript` execution context providing `object_world`.
      params: Script parameters with `workpiece_object_name`,
        `tool_object_name`, `tool_frame_name`, `workpiece_frame_name` (saved by
        `save_workpiece_at_grasp`), `grip_grasp_frame_name` and
        `grip_pregrasp_frame_name` (the grasp the part is held with), and
        `return_grasp_frame_name` and `return_pregrasp_frame_name` (written).
  """
  import math

  from intrinsic.math.python import data_types

  world = context.object_world
  workpiece = world.get_object(params.workpiece_object_name)
  tool = world.get_frame(params.tool_frame_name, params.tool_object_name)
  target = world.get_frame(params.workpiece_frame_name, "root")
  world_t_tool = world.get_transform(world.root, target) * world.get_transform(
    workpiece, tool
  )
  grip_t_pregrasp = world.get_transform(
    world.get_frame(params.grip_grasp_frame_name, "root"),
    world.get_frame(params.grip_pregrasp_frame_name, "root"),
  )
  pregrasp_distance_m = math.sqrt(
    sum(float(x) ** 2 for x in grip_t_pregrasp.translation)
  )
  pregrasp = world_t_tool * data_types.Pose3(
    translation=[0, 0, -pregrasp_distance_m]
  )
  _set_frame(world, params.return_grasp_frame_name, world.root, world_t_tool)
  _set_frame(world, params.return_pregrasp_frame_name, world.root, pregrasp)
