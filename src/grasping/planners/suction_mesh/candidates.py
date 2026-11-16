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

"""Top-down suction grasp candidates from a triangle mesh.

Places the suction cup on a grid over the part, seen from above. A cup
position is kept if the surface under the whole cup is nearly level and flat
enough to seal: vertical rays at the cup's center and around its rim must all
hit upward-facing faces whose heights differ by at most a tolerance.
Reachability and collisions are not checked here; the
`suction_mesh_plan_grasp` skill checks each candidate with the motion planner.
The mesh is expected to be validated by `src.grasping.workpiece_mesh`.
"""

import dataclasses

import numpy as np

from src.grasping import workpiece_mesh

# Rays per cup: the center, and points on two rings around it.
_RING_FRACTIONS = (0.5, 1.0)
_RAYS_PER_RING = 12


@dataclasses.dataclass(frozen=True)
class Candidate:
  """A top-down suction grasp, expressed in the world frame.

  Attributes:
    grasp: 4x4 pose of the tool frame with the cup pressed onto the surface.
    pregrasp: 4x4 pose of the tool frame above `grasp`.
    hover: 4x4 pose of the tool frame with the cup just above the surface,
      used to check the approach without touching the part.
    flatness_m: Height difference of the surface under the cup.
  """

  grasp: np.ndarray
  pregrasp: np.ndarray
  hover: np.ndarray
  flatness_m: float


def _cup_offsets(radius):
  """Returns (K, 2) XY offsets of the rays under a cup of this radius."""
  angles = np.linspace(0.0, 2 * np.pi, _RAYS_PER_RING, endpoint=False)
  ring = np.column_stack([np.cos(angles), np.sin(angles)])
  return np.vstack(
    [np.zeros((1, 2))] + [ring * radius * f for f in _RING_FRACTIONS]
  )


def top_down_grasps(
  vertices,
  triangles,
  world_t_object,
  gripper,
  *,
  pregrasp_distance_m=0.1,
  press_depth_m=0.001,
  hover_clearance_m=0.004,
  max_tilt_deg=15.0,
  flatness_tolerance_m=0.002,
  grid_step_m=0.004,
  max_candidates=20,
  preferred_x_axis=(1.0, 0.0, 0.0),
):
  """Returns suction grasp candidates in the world frame, best first.

  Candidates are ranked by the horizontal distance of the cup from the part's
  center of mass, assuming uniform density, so the part hangs as evenly as
  possible, then by flatness.

  Args:
    vertices: (N, 3) mesh vertices in the object frame.
    triangles: (M, 3) vertex indices, closed and outward facing.
    world_t_object: 4x4 pose of the object in the world.
    gripper: Cup geometry, an `omts.grasping.SuctionGripper`.
    pregrasp_distance_m: Height of the pregrasp above the grasp.
    press_depth_m: How far the grasp presses the cup below the surface, so the
      cup touches the part.
    hover_clearance_m: Gap between the cup and the surface at `hover`.
    max_tilt_deg: Largest angle between a face under the cup and the
      horizontal plane.
    flatness_tolerance_m: Largest height difference under the cup.
    grid_step_m: Spacing of the cup positions tried.
    max_candidates: Maximum number of candidates to return.
    preferred_x_axis: World direction the tool x axis should be near, such as
      the current tool x axis, to avoid unnecessary wrist rotation. The cup is
      symmetric about the tool z axis, so any yaw works.
  """
  rotation, translation = world_t_object[:3, :3], world_t_object[:3, 3]
  corners = vertices[triangles] @ rotation.T + translation
  normals = np.cross(
    corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]
  )
  normals /= np.linalg.norm(normals, axis=1)[:, None]
  points = corners.reshape(-1, 3)
  radius = gripper.cup_diameter_m / 2

  low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
  xs = np.arange(low[0], high[0] + 1e-9, grid_step_m)
  ys = np.arange(low[1], high[1] + 1e-9, grid_step_m)
  centers = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
  if not len(centers):
    return []
  offsets = _cup_offsets(radius)
  xy = (centers[:, None, :] + offsets[None]).reshape(-1, 2)
  top = points[:, 2].max() + 0.01
  origins = np.column_stack([xy, np.full(len(xy), top)])
  down = np.array([0.0, 0.0, -1.0])

  heights = np.full(len(xy), -np.inf)
  upright = np.zeros(len(xy), dtype=bool)
  min_up = np.cos(np.radians(max_tilt_deg))
  for i in range(0, len(xy), 64):
    distances = workpiece_mesh.ray_hits(
      origins[i : i + 64], np.tile(down, (len(origins[i : i + 64]), 1)), corners
    )
    nearest = np.argmin(distances, axis=1)
    hit = distances[np.arange(len(nearest)), nearest]
    heights[i : i + 64] = top - hit
    upright[i : i + 64] = np.isfinite(hit) & (normals[nearest, 2] >= min_up)

  heights = heights.reshape(len(centers), -1)
  upright = upright.reshape(len(centers), -1).all(axis=1)
  # A ray that misses the part leaves the cup unsealed.
  flatness = np.full(len(centers), np.inf)
  flatness[upright] = np.ptp(heights[upright], axis=1)
  keep = flatness <= flatness_tolerance_m

  centroid = workpiece_mesh.center_of_mass(corners)
  offset = np.linalg.norm(centers - centroid[:2], axis=1)
  order = np.lexsort((flatness, offset))

  x_axis = np.array(preferred_x_axis, dtype=float)
  x_axis[2] = 0.0
  if np.linalg.norm(x_axis) < 1e-6:
    x_axis = np.array([1.0, 0.0, 0.0])
  x_axis /= np.linalg.norm(x_axis)
  rotation = np.column_stack([x_axis, np.cross(down, x_axis), down])

  def pose(xy, z):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [xy[0], xy[1], z]
    return matrix

  candidates = []
  for i in order[keep[order]][:max_candidates]:
    surface = heights[i].max()
    # The tool frame sits `contact_depth_m` above the cup face.
    face_to_tool = gripper.contact_depth_m
    grasp = pose(centers[i], surface - press_depth_m + face_to_tool)
    hover = pose(centers[i], surface + hover_clearance_m + face_to_tool)
    pregrasp = grasp.copy()
    pregrasp[2, 3] += pregrasp_distance_m
    candidates.append(Candidate(grasp, pregrasp, hover, float(flatness[i])))
  return candidates
