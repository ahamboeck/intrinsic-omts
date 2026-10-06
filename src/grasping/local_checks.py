"""Conservative continuous object/end-effector checks, not robot feasibility.

Fixed rotations and linear translations let each complete approach/closure
sweep be enclosed by one oriented box per collider. Triangle clipping detects
surface intersection; solid-angle containment also detects a box fully inside
the object. These envelopes may reject feasible grasps. No discrete trajectory
samples, object AABB substitution or blanket gripper/object exemptions are used.

Assumes the same non-self-intersecting, outward single-shell mesh as search.
The Hand-E snapshot encloses SDF colliders, NOT certified physical hardware.
"""

import math
from dataclasses import dataclass

from src.grasping.contact_pairs import _prepare
from src.grasping.geometry import (
  RigidTransform,
  add,
  cross,
  dot,
  scale,
  sub,
  unit,
)
from src.grasping.poses import GraspPoseProposal
from src.grasping.profile import CollisionBox, ParallelJawProfile
from src.grasping.types import (
  UNIT_NORMAL_TOLERANCE,
  SurfaceContact,
  TriangleMesh,
  Vec3,
)

# Absolute floating-point allowances in metres. This is not a configurable
# penetration budget: only the designated inward pad plane may touch, and only
# with a matching outward object normal. Other ambiguous/touching hits reject.
_CONTACT_TOLERANCE_M = 1e-10
_NORMAL_TOLERANCE = 1e-9


@dataclass(frozen=True)
class LocalCheckResult:
  """Evidence about declared local geometry only; never 'safe to execute'."""

  accepted: bool
  reason: str
  stage: str = ""
  collider: str = ""


def _same_transform(first: RigidTransform, second: RigidTransform) -> bool:
  return math.dist(
    first.translation, second.translation
  ) <= _CONTACT_TOLERANCE_M and all(
    math.dist(a, b) <= _NORMAL_TOLERANCE
    for a, b in zip(first.rotation, second.rotation, strict=True)
  )


def _verify_contact(mesh: TriangleMesh, contact: SurfaceContact) -> None:
  if contact.triangle_index >= len(mesh.triangles):
    raise ValueError("contact triangle index is outside the source mesh.")
  a, b, c = (mesh.vertices[i] for i in mesh.triangles[contact.triangle_index])
  ab, ac, ap = sub(b, a), sub(c, a), sub(contact.point, a)
  normal = unit(cross(unit(ab), unit(ac)))
  if (
    abs(dot(ap, normal)) > _CONTACT_TOLERANCE_M
    or math.dist(normal, contact.outward_normal) > UNIT_NORMAL_TOLERANCE
  ):
    raise ValueError(
      "contact must lie on its source face with its outward normal."
    )
  # Scale the barycentric calculation to avoid overall mesh-size dependence.
  factor = max(math.hypot(*ab), math.hypot(*ac))
  ab, ac, ap = (scale(v, 1 / factor) for v in (ab, ac, ap))
  aa, cc, bc = dot(ab, ab), dot(ac, ac), dot(ab, ac)
  denominator = aa * cc - bc * bc
  if denominator <= 0:
    raise ValueError("contact face is numerically ill-conditioned.")
  u = (cc * dot(ap, ab) - bc * dot(ap, ac)) / denominator
  v = (aa * dot(ap, ac) - bc * dot(ap, ab)) / denominator
  if min(u, v, 1 - u - v) < -1e-9:
    raise ValueError("contact must belong to its source triangle.")


def _clip_triangle(vertices: tuple[Vec3, ...], half: Vec3) -> list[Vec3]:
  """Clip against six box half-spaces; boundary intersections are retained."""
  polygon = list(vertices)
  for axis in range(3):
    for sign in (-1, 1):
      if not polygon:
        return []
      result = []
      previous = polygon[-1]
      previous_distance = sign * previous[axis] - half[axis]
      for current in polygon:
        current_distance = sign * current[axis] - half[axis]
        if (current_distance <= 0) != (previous_distance <= 0):
          fraction = previous_distance / (previous_distance - current_distance)
          result.append(add(previous, scale(sub(current, previous), fraction)))
        if current_distance <= 0:
          result.append(current)
        previous, previous_distance = current, current_distance
      polygon = result
  return polygon


def _inside_object(
  point: Vec3, triangles: tuple[tuple[Vec3, ...], ...]
) -> bool:
  """Winding solid angle: also handles a collider with no surface crossings."""
  angles = []
  for triangle in triangles:
    directions = []
    for vertex in triangle:
      delta = sub(vertex, point)
      if math.hypot(*delta) <= _CONTACT_TOLERANCE_M:
        return True  # Boundary/uncertain is not certified free space.
      directions.append(unit(delta))
    a, b, c = directions
    angles.append(
      2 * math.atan2(dot(a, cross(b, c)), 1 + dot(a, b) + dot(b, c) + dot(c, a))
    )
  winding = math.fsum(angles)
  # For the supported embedded shell the values are 0 or +/-4*pi. Leave a
  # generous rejection interval instead of claiming near-boundary certainty.
  return not math.isfinite(winding) or abs(winding) > math.pi


def _sweep(
  box: CollisionBox, start: Vec3, finish: Vec3
) -> tuple[RigidTransform, Vec3]:
  """Collider-axis bounding box enclosing the whole straight translation."""
  midpoint = add(
    box.center_T_box.translation, add(scale(start, 0.5), scale(finish, 0.5))
  )
  displacement = sub(finish, start)
  half = tuple(
    extent + abs(dot(axis, displacement)) / 2
    for extent, axis in zip(
      box.half_extents_m, box.center_T_box.rotation, strict=True
    )
  )
  return RigidTransform(box.center_T_box.rotation, midpoint), half


def _hits_object(
  triangles: tuple[tuple[Vec3, ...], ...],
  sweep_T_box: RigidTransform,
  half: Vec3,
  contact_sign: int,
) -> bool:
  inverse = sweep_T_box.inverse()
  # Inflate the clipping volume to reject numerical near misses. Only the
  # allowed inward pad face has a narrowly defined numerical contact allowance.
  expanded = tuple(extent + _CONTACT_TOLERANCE_M / 4 for extent in half)
  for triangle in triangles:
    vertices = tuple(inverse.apply(vertex) for vertex in triangle)
    intersection = _clip_triangle(vertices, expanded)
    if not intersection:
      continue
    if contact_sign:
      inner_face = -contact_sign * half[0]
      normal = unit(
        cross(sub(vertices[1], vertices[0]), sub(vertices[2], vertices[0]))
      )
      if contact_sign * normal[0] >= 1 - _NORMAL_TOLERANCE and all(
        abs(point[0] - inner_face) <= _CONTACT_TOLERANCE_M
        for point in intersection
      ):
        continue
    return True
  return _inside_object(sweep_T_box.translation, triangles)


def check_local_grasp(
  mesh: TriangleMesh, proposal: GraspPoseProposal, profile: ParallelJawProfile
) -> LocalCheckResult:
  """Check the FULL declared open approach and closure to first pad contact.

  Does not model commanded overtravel after contact, friction/compliance,
  arm/table/fixture collisions, attached lift, or motion planning. The downstream
  adapter must verify commanded aperture and contact-stop behavior separately.
  Malformed/contradictory inputs raise ValueError; local incompatibility rejects.
  """
  if (
    not isinstance(mesh, TriangleMesh)
    or not isinstance(proposal, GraspPoseProposal)
    or not isinstance(profile, ParallelJawProfile)
  ):
    raise ValueError("require numeric mesh, pose proposal and gripper profile.")
  # Share the search package's supported topology/winding checks without running
  # random search or duplicating a subtly different mesh-admissibility contract.
  _prepare(mesh)
  for contact in (proposal.contacts.first, proposal.contacts.second):
    _verify_contact(mesh, contact)
  width = proposal.contacts.separation_m
  if (
    not profile.limits.min_contact_separation_m
    <= width
    < proposal.open_width_m
    <= profile.limits.max_opening_m
  ):
    raise ValueError("proposal aperture is inconsistent with contacts/profile.")
  center = proposal.object_T_center
  expected_midpoint = add(
    scale(proposal.contacts.first.point, 0.5),
    scale(proposal.contacts.second.point, 0.5),
  )
  closing = unit(
    sub(proposal.contacts.second.point, proposal.contacts.first.point)
  )
  if (
    math.dist(center.translation, expected_midpoint) > _CONTACT_TOLERANCE_M
    or math.dist(center.rotation[0], closing) > _NORMAL_TOLERANCE
  ):
    raise ValueError("center must match the contact midpoint and closing line.")
  if not _same_transform(
    proposal.object_T_tcp, center.compose(profile.center_T_tcp)
  ):
    raise ValueError("TCP pose is inconsistent with the profile calibration.")
  pregrasp = proposal.object_T_pregrasp_tcp
  delta = sub(pregrasp.translation, proposal.object_T_tcp.translation)
  distance = -dot(delta, center.rotation[2])
  if (
    distance <= 0
    or math.dist(delta, scale(center.rotation[2], -distance))
    > _CONTACT_TOLERANCE_M
    or any(
      math.dist(a, b) > _NORMAL_TOLERANCE
      for a, b in zip(
        pregrasp.rotation, proposal.object_T_tcp.rotation, strict=True
      )
    )
  ):
    raise ValueError(
      "pregrasp must translate opposite approach without rotation."
    )
  for sign, contact in (
    (-1, proposal.contacts.first),
    (1, proposal.contacts.second),
  ):
    if (
      dot(contact.outward_normal, scale(closing, sign)) < 1 - _NORMAL_TOLERANCE
    ):
      return LocalCheckResult(
        False,
        "contact normal does not match the modeled inward pad face",
        "contacts",
      )
  inverse = center.inverse()
  vertices = tuple(inverse.apply(vertex) for vertex in mesh.vertices)
  triangles = tuple(tuple(vertices[i] for i in face) for face in mesh.triangles)
  groups = (
    (0, profile.body_boxes),
    (1, profile.positive_jaw_boxes),
    (-1, profile.negative_jaw_boxes),
  )
  for stage in ("approach", "closure"):
    for sign, boxes in groups:
      open_offset = (
        sign * (proposal.open_width_m - profile.limits.max_opening_m) / 2,
        0,
        0,
      )
      closed_offset = (sign * (width - profile.limits.max_opening_m) / 2, 0, 0)
      start = (
        add(open_offset, (0, 0, -distance))
        if stage == "approach"
        else open_offset
      )
      finish = open_offset if stage == "approach" else closed_offset
      for box in boxes:
        sweep_pose, half = _sweep(box, start, finish)
        contact_sign = sign if stage == "closure" and box.is_contact_pad else 0
        if _hits_object(triangles, sweep_pose, half, contact_sign):
          return LocalCheckResult(
            False, "collision with conservative swept envelope", stage, box.name
          )
  return LocalCheckResult(True, "passed declared local checks")
