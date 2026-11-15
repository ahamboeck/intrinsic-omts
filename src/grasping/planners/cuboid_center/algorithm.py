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

"""Cuboid-center grasp, run as a behavior tree `PythonScript`.

The module is sent to the executive as source code, so it imports only what
the script runtime provides.
"""

import math
from typing import Any

from intrinsic.math.python import data_types


def plan_cuboid_center_grasp(context: Any, params: Any) -> None:
  """Writes a top-down grasp at the center of the workpiece's pose.

  The jaws close across the part's longest horizontal axis: local X, or local
  Z if X points mostly up. Of the four top-down orientations about that axis,
  the one closest to the current tool orientation is used, to avoid
  unnecessary wrist rotation. The pregrasp is `approach_offset_z` above the
  grasp.

  Args:
      context: `PythonScript` execution context providing `object_world`.
      params: Script parameters with `parent_object`, `workpiece_object_name`,
        `tool_object_name`, `tool_frame_name`, `approach_offset_z`,
        `grasp_frame_name` and `pregrasp_frame_name`.

  Raises:
      ValueError: If the workpiece is not in the world.
  """
  world = context.object_world
  parent_obj = getattr(
    world, params.parent_object, getattr(world, "root", None)
  )
  workpiece = getattr(world, params.workpiece_object_name, None)
  if workpiece is None:
    raise ValueError(
      f"Workpiece object '{params.workpiece_object_name}' not found in world."
    )
  parent_t_workpiece = world.get_transform(parent_obj, workpiece)
  target_pos = parent_t_workpiece.translation
  target_rot = parent_t_workpiece.rotation

  # Local X is the longest axis (5"), local Z is the medium axis (3"), local Y
  # is the thickness (2").
  ax = target_rot.rotate_point([1.0, 0.0, 0.0])
  az = target_rot.rotate_point([0.0, 0.0, 1.0])

  if abs(float(ax[2])) < 0.7:
    vx, vy = float(ax[0]), float(ax[1])
  else:
    vx, vy = float(az[0]), float(az[1])

  psi = math.atan2(vy, vx)
  half_psi = psi / 2.0
  c1 = (math.cos(half_psi), math.sin(half_psi), 0.0, 0.0)
  c2 = (-math.sin(half_psi), math.cos(half_psi), 0.0, 0.0)
  orientations = [
    c1,
    (-c1[0], -c1[1], 0.0, 0.0),
    c2,
    (-c2[0], -c2[1], 0.0, 0.0),
  ]

  tool_obj = getattr(world, str(params.tool_object_name), None)
  tool_node = (
    getattr(tool_obj, str(params.tool_frame_name), None) if tool_obj else None
  )
  cur_tool_tf = (
    world.get_transform(parent_obj, tool_node) if tool_node else None
  )

  if cur_tool_tf is not None:
    cur_q = cur_tool_tf.rotation.quaternion
    grasp_ori = max(
      orientations,
      key=lambda c: (
        c[0] * float(cur_q.x)
        + c[1] * float(cur_q.y)
        + c[2] * float(cur_q.z)
        + c[3] * float(cur_q.w)
      ),
    )
  else:
    grasp_ori = c1

  grasp_pos = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
  pregrasp_pos = (
    grasp_pos[0],
    grasp_pos[1],
    grasp_pos[2] + params.approach_offset_z,
  )

  frame_poses = {
    params.pregrasp_frame_name: (pregrasp_pos, grasp_ori),
    params.grasp_frame_name: (grasp_pos, grasp_ori),
  }

  has_list_frames = hasattr(parent_obj, "list_frames")
  existing_frames = set()
  if has_list_frames:
    existing_frames = set(parent_obj.list_frames())
  elif hasattr(parent_obj, "__dict__"):
    existing_frames = set(parent_obj.__dict__.keys())

  for fname, (pos, ori) in frame_poses.items():
    pose = data_types.Pose3(
      data_types.Rotation3(
        data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])
      ),
      [pos[0], pos[1], pos[2]],
    )
    frame_exists = fname in existing_frames or (
      not has_list_frames and hasattr(parent_obj, fname)
    )
    if frame_exists:
      frame_node = getattr(parent_obj, fname)
      world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
    else:
      world.create_frame(
        frame_name=fname, parent=parent_obj, parent_t_frame=pose
      )
