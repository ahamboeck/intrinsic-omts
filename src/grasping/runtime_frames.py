"""Runtime local-only mesh planning for the opt-in simulation pick pipeline.

No simulator state, cached grasp fallback, or motion execution. Core move_robot
checks subsequent motions; these proposals are NOT full robot-feasible selection.
"""

import json
import math
import statistics

import numpy as np
from intrinsic.math.python import data_types

from src.grasping.contact_pairs import propose_contact_pairs
from src.grasping.geometry import RigidTransform, dot, sub, unit
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.local_checks import check_local_grasp
from src.grasping.poses import propose_top_down_grasps
from src.grasping.types import ContactPairOptions, TriangleMesh


def mesh_from_json(cad):
  """Reindex exact CAD export seams, never approximate or replace with bounds."""
  vertices, indices, remap = [], {}, []
  for value in cad["vertices"]:
    point = tuple(value)
    if point not in indices:
      indices[point] = len(vertices)
      vertices.append(point)
    remap.append(indices[point])
  return TriangleMesh(
    tuple(vertices),
    tuple(tuple(remap[i] for i in face) for face in cad["triangles"]),
  )


def from_sdk(pose):
  return RigidTransform(
    tuple(
      tuple(float(x) for x in pose.rotation.rotate_point(axis))
      for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    ),
    tuple(float(x) for x in pose.translation),
  )


def to_sdk(pose):
  rows = [[pose.rotation[c][r] for c in range(3)] for r in range(3)]
  return data_types.Pose3(
    data_types.Rotation3.from_matrix(np.array(rows)), list(pose.translation)
  )


def top_down_direction_filter(world_t_object, max_tilt_deg):
  """Object-unit closing directions permitting world-down approach, no snapping."""
  world_down = unit(world_t_object.inverse().rotate((0, 0, -1)))
  limit = math.sin(math.radians(max_tilt_deg))

  def compatible(direction):
    vertical = abs(dot(unit(direction), world_down))
    return vertical < 1 and vertical <= limit + 1e-12

  return compatible


def contact_pair_diagnostics(
  pairs, world_t_object, profile, open_width_m, max_tilt_deg
):
  """Finite sampled evidence, not an exhaustive feasibility claim.

  Required approach tilt is asin(abs(world closing line Z)): approach must be
  perpendicular to the unchanged contact line. Widths are reported in mm,
  angles in degrees; rotations and contacts remain in their original frames.
  """
  widths, tilts = [], []
  rejected_width, rejected_tilt, compatible = 0, 0, 0
  accepts_direction = top_down_direction_filter(world_t_object, max_tilt_deg)
  for pair in pairs:
    direction = unit(sub(pair.second.point, pair.first.point))
    world_direction = world_t_object.rotate(direction)
    widths.append(pair.separation_m * 1000)
    tilts.append(math.degrees(math.asin(min(1.0, abs(world_direction[2])))))
    if (
      not profile.limits.min_contact_separation_m
      <= pair.separation_m
      < open_width_m
    ):
      rejected_width += 1
    elif not accepts_direction(direction):
      rejected_tilt += 1
    else:
      compatible += 1

  def summary(values):
    return (
      None
      if not values
      else dict(
        min=min(values), median=statistics.median(values), max=max(values)
      )
    )

  return dict(
    contact_pairs=len(pairs),
    contact_width_mm=summary(widths),
    required_approach_tilt_deg=summary(tilts),
    rejected_open_width=rejected_width,
    rejected_approach_tilt=rejected_tilt,
    approach_compatible_pairs=compatible,
  )


def plan_mesh_frames(context, params):
  """Read the preceding perception's belief pose; publish only a local pass."""
  world = context.object_world
  if params.parent_object != "root":
    raise ValueError(
      "Mesh simulation backend currently requires root output frames."
    )
  parent = world.root
  obj = world.get_object(params.workpiece_object_name)
  world_t_object = from_sdk(world.get_transform(parent, obj))
  mesh = mesh_from_json(json.loads(params.cad_json))
  if len(mesh.triangles) > 10000:
    raise ValueError(
      "Mesh exceeds 10000 triangles; no automatic simplification."
    )
  profile = hande_collision_profile()
  options = ContactPairOptions(
    seed=42,
    max_pairs=500,
    max_samples=20000,
    opening_clearance_m=0.001,
  )
  baseline_pairs = propose_contact_pairs(mesh, profile.limits, options=options)
  pairs = propose_contact_pairs(
    mesh,
    profile.limits,
    options=options,
    closing_direction_filter=top_down_direction_filter(
      world_t_object,
      params.max_approach_tilt_deg,
    ),
  )
  poses = propose_top_down_grasps(
    tuple(pairs),
    profile,
    world_T_object=world_t_object,
    pregrasp_distance_m=params.pregrasp_distance_m,
    open_width_m=0.06,
    max_approach_tilt_rad=math.radians(params.max_approach_tilt_deg),
  )
  diagnostics = dict(
    baseline=contact_pair_diagnostics(
      baseline_pairs,
      world_t_object,
      profile,
      0.06,
      params.max_approach_tilt_deg,
    ),
    focused=contact_pair_diagnostics(
      pairs, world_t_object, profile, 0.06, params.max_approach_tilt_deg
    ),
    max_approach_tilt_deg=params.max_approach_tilt_deg,
    proposals=len(poses),
    world_object_rotation_basis_columns=world_t_object.rotation,
  )
  print("Mesh search diagnostics: " + json.dumps(diagnostics, sort_keys=True))
  tool = world.get_frame(params.tool_frame_name, params.tool_object_name)
  tool_pose = from_sdk(world.get_transform(parent, tool))
  center = tuple(
    (min(p[i] for p in mesh.vertices) + max(p[i] for p in mesh.vertices)) / 2
    for i in range(3)
  )

  def rank(pose):
    world_pose = world_t_object.compose(pose.object_T_tcp)
    # Prefer contacts near the CAD center, then the closest current wrist yaw.
    alignment = sum(
      sum(a * b for a, b in zip(x, y, strict=True))
      for x, y in zip(world_pose.rotation, tool_pose.rotation, strict=True)
    )
    return (
      math.dist(pose.object_T_center.translation, center),
      -alignment,
      pose.object_T_tcp.translation,
      pose.object_T_tcp.rotation,
    )

  selected = None
  for pose in sorted(poses, key=rank):
    if check_local_grasp(mesh, pose, profile).accepted:
      selected = pose
      break
  if selected is None:
    raise ValueError(
      f"Mesh planning rejected current perception: {len(pairs)} contact pairs, "
      f"{len(poses)} proposals, no locally accepted grasp. No fallback."
      " Diagnostics: " + json.dumps(diagnostics, sort_keys=True)
    )
  # Resolve both destinations before writing either. A failed script stops the
  # surrounding Sequence, so stale frames can never drive this pick on failure.
  grasp = world.get_frame(params.grasp_frame_name, params.parent_object)
  pregrasp = world.get_frame(params.pregrasp_frame_name, params.parent_object)
  for frame, pose in (
    (grasp, selected.object_T_tcp),
    (pregrasp, selected.object_T_pregrasp_tcp),
  ):
    world.update_transform(
      node_a=parent, node_b=frame, a_t_b=to_sdk(world_t_object.compose(pose))
    )
