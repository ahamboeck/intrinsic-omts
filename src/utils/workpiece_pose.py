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

"""Updates the detected workpiece's pose in the world, run as a `PythonScript`.

Grasp planning is not part of perception: the selected grasp planner plans on
the pose written here.
"""

from typing import Any

from intrinsic.math.python import data_types


def update_detected_workpiece_pose(context: Any, params: Any) -> None:
  """Writes a pose estimate from the camera frame into the world.

  Composes the estimate with the camera sensor's pose in `parent_object`,
  rejects poses below `min_safe_z`, and updates the detected workpiece's pose
  if `target_scene_object_id` names an object in the world.

  Args:
      context: `PythonScript` execution context providing `object_world`.
      params: Script parameters with the estimate (`pos_*`, `ori_*`) in the
        camera frame, `parent_object`, `camera_name`, `target_scene_object_id`
        and `min_safe_z`.

  Raises:
      ValueError: If the camera is not in the world, or the estimate is below
        `min_safe_z`.
  """
  world = context.object_world
  parent_obj = getattr(
    world, params.parent_object, getattr(world, "root", None)
  )

  # Resolve camera sensor transform in parent object (root)
  camera_obj = getattr(world, params.camera_name, None)
  if camera_obj is None:
    raise ValueError(
      f"Camera object '{params.camera_name}' not found in world."
    )

  camera_sensor_node = (
    getattr(camera_obj, "sensor", camera_obj) if camera_obj else None
  )
  root_t_camera = (
    world.get_transform(parent_obj, camera_sensor_node)
    if camera_sensor_node
    else None
  )

  # Construct detected pose in camera frame
  cam_q = data_types.Quaternion(
    [params.ori_x, params.ori_y, params.ori_z, params.ori_w]
  )
  cam_pose = data_types.Pose3(
    data_types.Rotation3(cam_q), [params.pos_x, params.pos_y, params.pos_z]
  )

  if root_t_camera is not None:
    root_t_target = root_t_camera * cam_pose
  else:
    root_t_target = cam_pose

  target_pos = root_t_target.translation

  min_safe_z = float(params.min_safe_z)
  if float(target_pos[2]) < min_safe_z:
    raise ValueError(
      f"Calculated workpiece target Z ({float(target_pos[2]):.4f}m) is below "
      f"minimum safe height min_safe_z ({min_safe_z:.4f}m)."
    )

  # Update world object pose if target_scene_object_id is present
  target_id = getattr(params, "target_scene_object_id", "")
  if target_id and hasattr(world, "update_transform"):
    target_obj = getattr(world, target_id, None) or getattr(
      world, target_id.split(".")[-1], None
    )
    if target_obj is not None:
      actual_parent = (
        getattr(target_obj, "parent", None)
        or getattr(target_obj, "parent_object", None)
        or parent_obj
      )
      world.update_transform(
        node_a=actual_parent, node_b=target_obj, a_t_b=root_t_target
      )
