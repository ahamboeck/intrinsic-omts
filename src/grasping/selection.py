"""Bounded geometric ranking and selection through an explicit feasibility gate.

No SDK, world mutation, actuator command, freshness assertion or IK substitute.
The runtime must supply fresh localized input and a real downstream checker.
"""

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

from src.grasping.contact_pairs import propose_contact_pairs
from src.grasping.geometry import RigidTransform, dot, sub
from src.grasping.local_checks import check_local_grasp
from src.grasping.poses import GraspPoseProposal, propose_top_down_grasps
from src.grasping.profile import ParallelJawProfile
from src.grasping.types import (
  ContactPairOptions,
  TriangleMesh,
  _require_finite_number,
  _require_integer,
)


@dataclass(frozen=True)
class SelectionOptions:
  contact_options: ContactPairOptions
  pregrasp_distance_m: float
  open_width_m: float
  max_feasibility_checks: int
  max_triangles: int = 10000
  position_tolerance_m: float = 0.001
  rotation_tolerance_rad: float = math.radians(1)

  def __post_init__(self) -> None:
    if not isinstance(self.contact_options, ContactPairOptions):
      raise ValueError("contact_options must be ContactPairOptions.")
    for name in ("max_feasibility_checks", "max_triangles"):
      _require_integer(getattr(self, name), name, minimum=1)
    for name in (
      "pregrasp_distance_m",
      "open_width_m",
      "position_tolerance_m",
      "rotation_tolerance_rad",
    ):
      value = getattr(self, name)
      _require_finite_number(value, name)
      if value <= 0:
        raise ValueError(f"{name} must be positive.")
    if self.rotation_tolerance_rad >= math.pi:
      raise ValueError("rotation_tolerance_rad must be less than pi.")


@dataclass(frozen=True)
class FeasibilityResult:
  """Runtime checker result for pregrasp, contact approach AND attached lift.

  An endpoint-only IK check must not return accepted=True. A runtime checker
  must enforce its RPC deadlines and collision policy, not execute motion.
  """

  accepted: bool
  reason: str = ""

  def __post_init__(self) -> None:
    if type(self.accepted) is not bool or not isinstance(self.reason, str):
      raise ValueError("require bool accepted and string reason.")
    if not self.accepted and not self.reason:
      raise ValueError("rejected feasibility requires a reason.")


@dataclass(frozen=True)
class CandidateRejection:
  candidate_index: int
  stage: str
  reason: str


@dataclass(frozen=True)
class SelectionReport:
  selected: GraspPoseProposal | None
  contact_pairs: int
  pose_proposals: int
  feasibility_checks: int
  rejections: tuple[CandidateRejection, ...]


def _similar(
  a: RigidTransform, b: RigidTransform, options: SelectionOptions
) -> bool:
  if (
    math.hypot(*sub(a.translation, b.translation))
    > options.position_tolerance_m
  ):
    return False
  cosine = (
    sum(dot(x, y) for x, y in zip(a.rotation, b.rotation, strict=True)) - 1
  ) / 2
  return cosine >= math.cos(options.rotation_tolerance_rad)


def select_grasp(
  mesh: TriangleMesh,
  profile: ParallelJawProfile,
  *,
  world_T_object: RigidTransform,
  options: SelectionOptions,
  check_feasibility: Callable[
    [RigidTransform, RigidTransform, float], FeasibilityResult
  ],
  deadline: float,
  clock: Callable[[], float] = time.monotonic,
) -> SelectionReport:
  """Select an alternative only after local AND downstream acceptance.

  Rank by distance to the mesh bounding-box center, then higher object-local Z,
  then numeric pose tie-breakers. This is a geometric preference, not a stability
  score or success probability. Deduplicate full TCP poses; preserve distinct yaw.

  deadline is an absolute time in clock's monotonic domain. Checks between each
  bounded operation and after the callback discard late results. This cannot
  interrupt Python geometry work or a blocking callback; the caller must enforce
  RPC timeouts using the remaining-seconds argument. No previous pose is reused.
  """
  _require_finite_number(deadline, "deadline")
  if not isinstance(mesh, TriangleMesh) or not isinstance(
    options, SelectionOptions
  ):
    raise ValueError("require a mesh and SelectionOptions.")
  if not isinstance(profile, ParallelJawProfile) or not isinstance(
    world_T_object, RigidTransform
  ):
    raise ValueError("require a jaw profile and world_T_object transform.")
  if not callable(check_feasibility):
    raise ValueError("a downstream feasibility checker is required.")
  if len(mesh.triangles) > options.max_triangles:
    raise ValueError("mesh exceeds max_triangles; no automatic simplification.")

  def remaining() -> float:
    now = clock()
    _require_finite_number(now, "clock result")
    seconds = deadline - now
    if seconds <= 0:
      raise TimeoutError("mesh grasp selection deadline exceeded.")
    return seconds

  remaining()
  pairs = propose_contact_pairs(
    mesh, profile.limits, options=options.contact_options
  )
  remaining()
  poses = propose_top_down_grasps(
    tuple(pairs),
    profile,
    world_T_object=world_T_object,
    pregrasp_distance_m=options.pregrasp_distance_m,
    open_width_m=options.open_width_m,
  )
  center = tuple(
    (min(v[i] for v in mesh.vertices) + max(v[i] for v in mesh.vertices)) / 2
    for i in range(3)
  )

  def rank(pose: GraspPoseProposal) -> tuple:
    position = pose.object_T_center.translation
    return (
      math.hypot(*sub(position, center)),
      -position[2],
      position,
      pose.object_T_tcp.rotation,
    )

  ranked = sorted(poses, key=rank)
  seen = []
  rejections = []
  checks = 0
  selected = None
  for index, pose in enumerate(ranked):
    remaining()
    if any(_similar(pose.object_T_tcp, prior, options) for prior in seen):
      rejections.append(
        CandidateRejection(index, "duplicate", "near-identical TCP pose")
      )
      continue
    local = check_local_grasp(mesh, pose, profile)
    remaining()
    if not local.accepted:
      rejections.append(
        CandidateRejection(
          index, local.stage, f"{local.collider}: {local.reason}"
        )
      )
      continue
    # A rejected local pose must not suppress a nearby locally valid pose.
    seen.append(pose.object_T_tcp)
    if checks >= options.max_feasibility_checks:
      rejections.append(
        CandidateRejection(
          index, "budget", "feasibility check budget exhausted"
        )
      )
      break
    result = check_feasibility(
      world_T_object.compose(pose.object_T_tcp),
      world_T_object.compose(pose.object_T_pregrasp_tcp),
      remaining(),
    )
    checks += 1
    remaining()
    if not isinstance(result, FeasibilityResult):
      raise ValueError("downstream checker must return FeasibilityResult.")
    if result.accepted:
      selected = pose
      break
    rejections.append(CandidateRejection(index, "downstream", result.reason))
  remaining()
  return SelectionReport(
    selected, len(pairs), len(poses), checks, tuple(rejections)
  )
