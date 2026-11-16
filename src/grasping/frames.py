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

"""Writes planned grasp frames into the Object World from a skill."""

from intrinsic.math.python import data_types


def set_root_frames(world, poses):
  """Sets frames on `root` to world poses, creating the ones that are missing.

  Args:
    world: Object World client, such as a skill's `context.object_world`.
    poses: Frame name to its 4x4 pose in the world.
  """
  root = world.root
  for name, pose in poses.items():
    transform = data_types.Pose3.from_matrix4x4(pose)
    if name in root.frame_names:
      world.update_transform(
        node_a=root, node_b=world.get_frame(name, "root"), a_t_b=transform
      )
    else:
      world.create_frame(frame_name=name, parent=root, parent_t_frame=transform)
