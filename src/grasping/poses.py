"""World-top-down pose proposals, still requiring local and robot validation."""

import math
from dataclasses import dataclass

from src.grasping.geometry import (
  RigidTransform,
  add,
  cross,
  dot,
  scale,
  sub,
  unit,
)
from src.grasping.profile import ParallelJawProfile
from src.grasping.types import (
  ContactPair,
  _require_finite_number,
  _require_tuple,
)

DEFAULT_MAX_APPROACH_TILT_RAD = math.radians(2)


@dataclass(frozen=True)
class GraspPoseProposal:
  """Object-local center/TCP frames and declared open aperture, not a valid grasp."""

  contacts: ContactPair
  object_T_center: RigidTransform
  object_T_tcp: RigidTransform
  object_T_pregrasp_tcp: RigidTransform
  open_width_m: float

  def __post_init__(self) -> None:
    if not isinstance(self.contacts, ContactPair) or any(
      not isinstance(pose, RigidTransform)
      for pose in (
        self.object_T_center,
        self.object_T_tcp,
        self.object_T_pregrasp_tcp,
      )
    ):
      raise ValueError("proposal requires contacts and rigid transforms.")
    _require_finite_number(self.open_width_m, "open_width_m")
    if self.open_width_m <= 0:
      raise ValueError("open_width_m must be positive.")


def propose_top_down_grasps(
  pairs: tuple[ContactPair, ...],
  profile: ParallelJawProfile,
  *,
  world_T_object: RigidTransform,
  pregrasp_distance_m: float,
  open_width_m: float,
  max_approach_tilt_rad: float = DEFAULT_MAX_APPROACH_TILT_RAD,
) -> list[GraspPoseProposal]:
  """Build two yaw alternatives per compatible pair, without ranking or collision checks.

  world_T_object must be a fresh localized object pose. +X remains the actual
  contact closing line; +Z is the closest perpendicular direction to world down.
  Reject pairs requiring more than max_approach_tilt_rad (default 2 degrees).
  Zero requests strict top-down. Never project contacts or snap the object pose.
  +Y=Z cross X; pregrasp translates TCP opposite the bounded approach direction.
  open_width_m is the verified aperture of the intended open COMMAND, not
  automatically the profile's maximum. Command mapping belongs in the adapter.
  """
  _require_tuple(pairs, "pairs")
  if (
    any(not isinstance(pair, ContactPair) for pair in pairs)
    or not isinstance(profile, ParallelJawProfile)
    or not isinstance(world_T_object, RigidTransform)
  ):
    raise ValueError(
      "require contact pairs, numeric profile and world_T_object."
    )
  for value, name in (
    (pregrasp_distance_m, "pregrasp_distance_m"),
    (open_width_m, "open_width_m"),
  ):
    _require_finite_number(value, name)
    if value <= 0:
      raise ValueError(f"{name} must be positive.")
  if (
    not profile.limits.min_contact_separation_m
    <= open_width_m
    <= profile.limits.max_opening_m
  ):
    raise ValueError("open width is outside profile limits.")
  _require_finite_number(max_approach_tilt_rad, "max_approach_tilt_rad")
  if not 0 <= max_approach_tilt_rad < math.pi / 2:
    raise ValueError("max_approach_tilt_rad must be in [0, pi/2).")
  world_down = unit(world_T_object.inverse().rotate((0, 0, -1)))
  proposals = []
  for pair in pairs:
    if (
      not profile.limits.min_contact_separation_m
      <= pair.separation_m
      < open_width_m
    ):
      continue
    direction = unit(sub(pair.second.point, pair.first.point))
    vertical = dot(direction, world_down)
    # Only roundoff allowance, not an extra geometric tilt budget.
    if (
      abs(vertical) >= 1
      or abs(vertical) > math.sin(max_approach_tilt_rad) + 1e-12
    ):
      continue
    # Cross products avoid cancellation when projecting near a vertical line.
    approach = unit(cross(direction, unit(cross(world_down, direction))))
    center = add(scale(pair.first.point, 0.5), scale(pair.second.point, 0.5))
    for contacts, sign in (
      (pair, 1),
      (ContactPair(pair.second, pair.first), -1),
    ):
      x = scale(direction, sign)
      center_pose = RigidTransform(
        (x, unit(cross(approach, x)), approach), center
      )
      tcp = center_pose.compose(profile.center_T_tcp)
      pregrasp = RigidTransform(
        tcp.rotation, sub(tcp.translation, scale(approach, pregrasp_distance_m))
      )
      proposals.append(
        GraspPoseProposal(contacts, center_pose, tcp, pregrasp, open_width_m)
      )
  return proposals
