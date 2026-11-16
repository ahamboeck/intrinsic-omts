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

"""Top-down parallel-jaw grasp candidates from a triangle mesh.

Samples surface points, casts a ray from each one straight through the part to
find the opposite contact, and keeps pairs that a top-down parallel-jaw
gripper can close on. Reachability and collisions are not checked here; the
`antipodal_mesh_plan_grasp` skill checks each candidate with the motion planner.
The mesh is expected to be validated by `src.grasping.workpiece_mesh`.
"""

import dataclasses

import numpy as np

from src.grasping import workpiece_mesh


@dataclasses.dataclass(frozen=True)
class Candidate:
  """A top-down grasp, expressed in the world frame.

  Attributes:
    grasp: 4x4 pose of the tool frame at the contacts, jaws open.
    pregrasp: 4x4 pose of the tool frame above `grasp`.
    width_m: Distance between the two contacts.
  """

  grasp: np.ndarray
  pregrasp: np.ndarray
  width_m: float


def top_down_grasps(
  vertices,
  triangles,
  world_t_object,
  gripper,
  *,
  pregrasp_distance_m=0.1,
  max_tilt_deg=10.0,
  friction_cone_deg=15.0,
  num_samples=2000,
  max_candidates=20,
  preferred_jaw_axis=(1.0, 0.0, 0.0),
  seed=42,
):
  """Returns grasp candidates in the world frame, best first.

  Candidates are ranked by how close the grasp center is to the part's
  centroid (a stable, central hold), then by how directly the two surface
  normals oppose each other.

  Args:
    vertices: (N, 3) mesh vertices in the object frame.
    triangles: (M, 3) vertex indices, closed and outward facing.
    world_t_object: 4x4 pose of the object in the world.
    gripper: Jaw geometry, an `omts.grasping.ParallelJawGripper`.
    pregrasp_distance_m: Height of the pregrasp above the grasp.
    max_tilt_deg: Largest angle between a contact face's normal and the
      horizontal plane, which also absorbs tilt in `world_t_object`.
    friction_cone_deg: Largest angle between the jaw axis and either normal.
    num_samples: Surface points to sample.
    max_candidates: Maximum number of distinct candidates to return.
    preferred_jaw_axis: World direction the tool x axis should be near, such as
      the current tool x axis, to avoid unnecessary wrist rotation.
    seed: Random seed, so planning is repeatable.
  """
  rotation, translation = world_t_object[:3, :3], world_t_object[:3, 3]
  corners = vertices[triangles] @ rotation.T + translation
  normals = np.cross(
    corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]
  )
  areas = np.linalg.norm(normals, axis=1)
  normals /= np.where(areas > 0, areas, 1.0)[:, None]

  # Only faces whose normal is close to horizontal can carry a jaw contact.
  side = np.abs(normals[:, 2]) <= np.sin(np.radians(max_tilt_deg))
  if not side.any():
    return []
  rng = np.random.default_rng(seed)
  weights = np.where(side, areas, 0.0)
  faces = rng.choice(len(corners), size=num_samples, p=weights / weights.sum())
  a, b = rng.random((2, num_samples))
  flip = a + b > 1
  a, b = np.where(flip, 1 - a, a), np.where(flip, 1 - b, b)
  points = (
    corners[faces, 0]
    + a[:, None] * (corners[faces, 1] - corners[faces, 0])
    + b[:, None] * (corners[faces, 2] - corners[faces, 0])
  )
  inward = -normals[faces]

  first = np.empty(num_samples, dtype=int)
  width = np.empty(num_samples)
  for i in range(0, num_samples, 64):
    distances = workpiece_mesh.ray_hits(
      points[i : i + 64], inward[i : i + 64], corners
    )
    nearest = np.argmin(distances, axis=1)
    first[i : i + 64] = nearest
    width[i : i + 64] = distances[np.arange(len(nearest)), nearest]
  # The far face of a good pair faces along the ray, away from the near face.
  opposing = np.einsum("ij,ij->i", normals[first], inward)
  keep = (
    np.isfinite(width)
    & (width >= gripper.min_width_m)
    & (width <= gripper.max_width_m)
    & (opposing >= np.cos(np.radians(friction_cone_deg)))
  )

  centroid = corners.reshape(-1, 3).mean(axis=0)
  centers = points + inward * width[:, None] / 2
  offset = np.linalg.norm((centers - centroid)[:, :2], axis=1)
  order = np.lexsort((-opposing, offset))
  preferred = np.asarray(preferred_jaw_axis, dtype=float)

  candidates, seen = [], set()
  for i in order[keep[order]]:
    axis = np.array([inward[i, 0], inward[i, 1], 0.0])
    axis /= np.linalg.norm(axis)
    if axis @ preferred < 0:
      axis = -axis
    yaw = round(np.degrees(np.arctan2(axis[1], axis[0])) / 10) % 36
    key = (*np.round(centers[i] / 0.002).astype(int), yaw)
    if key in seen:
      continue
    seen.add(key)
    if not _closing_path_clear(
      points[i], inward[i], width[i], gripper, corners
    ):
      continue
    down = np.array([0.0, 0.0, -1.0])
    grasp = np.eye(4)
    grasp[:3, :3] = np.column_stack([axis, np.cross(down, axis), down])
    grasp[:3, 3] = centers[i] - gripper.contact_depth_m * down
    pregrasp = grasp.copy()
    pregrasp[2, 3] += pregrasp_distance_m
    candidates.append(Candidate(grasp, pregrasp, float(width[i])))
    if len(candidates) == max_candidates:
      break
  return candidates


def _closing_path_clear(contact, inward, width, gripper, corners):
  """True if each open fingertip reaches its contact without hitting the part.

  The fingertips start `max_width_m` apart, centered on the contacts, and
  close along the jaw axis. A surface between a fingertip and its contact
  would stop the finger before it reaches the planned contact.
  """
  travel = (gripper.max_width_m - width) / 2
  if travel <= 1e-6:
    return True
  far = contact + inward * width
  starts = np.stack([contact - inward * travel, far + inward * travel])
  directions = np.stack([inward, -inward])
  hits = workpiece_mesh.ray_hits(starts, directions, corners).min(axis=1)
  return bool((hits >= travel - 1e-4).all())
