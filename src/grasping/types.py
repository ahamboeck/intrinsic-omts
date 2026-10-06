"""Immutable numeric contracts; lengths and object-local points are in metres.

Construction validates structure, not mesh topology, surface membership,
antipodal alignment, collisions, robot feasibility or physical grasp stability.
"""

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Triangle = tuple[int, int, int]

# Absolute tolerance on normal length (not squared length), with no relative
# tolerance. Normals within tolerance are retained, not silently normalized.
UNIT_NORMAL_TOLERANCE = 1e-6


def _require_finite_number(value: float, name: str) -> None:
  if type(value) not in (int, float):
    raise ValueError(f"{name} must be a finite int or float, not bool.")
  try:
    finite = math.isfinite(value)
  except OverflowError:
    finite = False
  if not finite:
    raise ValueError(f"{name} must be finite and representable as a float.")


def _require_tuple(
  value: tuple, name: str, *, length: int | None = None
) -> None:
  if type(value) is not tuple or (length is not None and len(value) != length):
    suffix = f" of length {length}" if length is not None else ""
    raise ValueError(f"{name} must be a tuple{suffix}.")


def _require_vec3(value: Vec3, name: str) -> None:
  _require_tuple(value, name, length=3)
  for coordinate in value:
    _require_finite_number(coordinate, name)


def _require_integer(
  value: int, name: str, *, minimum: int | None = None
) -> None:
  if type(value) is not int or (minimum is not None and value < minimum):
    suffix = f" >= {minimum}" if minimum is not None else ""
    raise ValueError(f"{name} must be an integer{suffix}, not bool.")


@dataclass(frozen=True)
class TriangleMesh:
  """Indexed surface in object-local metres, with immutable tuple storage.

  Faces must be nondegenerate at floating-point precision. No minimum area is
  imposed. Closed-shell, winding and self-intersection checks belong to search.
  """

  vertices: tuple[Vec3, ...]
  triangles: tuple[Triangle, ...]

  def __post_init__(self) -> None:
    _require_tuple(self.vertices, "vertices")
    _require_tuple(self.triangles, "triangles")
    if not self.vertices or not self.triangles:
      raise ValueError("vertices and triangles must be nonempty.")
    for vertex in self.vertices:
      _require_vec3(vertex, "vertex")
    for triangle in self.triangles:
      _require_tuple(triangle, "triangle", length=3)
      for index in triangle:
        _require_integer(index, "triangle index", minimum=0)
        if index >= len(self.vertices):
          raise ValueError("triangle index is outside vertices.")
      a, b, c = (self.vertices[index] for index in triangle)
      edges = []
      for endpoint in (b, c):
        edge = tuple(
          float(end) - float(start)
          for start, end in zip(a, endpoint, strict=True)
        )
        length = math.hypot(*edge)
        if not math.isfinite(length) or length == 0:
          raise ValueError("triangle edges must have positive finite lengths.")
        # Normalize before the cross product to avoid an arbitrary area cutoff
        # and overflow/underflow caused solely by the overall mesh scale.
        edges.append(tuple(component / length for component in edge))
      u, v = edges
      cross = (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
      )
      if math.hypot(*cross) == 0:
        raise ValueError("triangles must be nondegenerate.")


@dataclass(frozen=True)
class ParallelJawLimits:
  """Aperture limits only; not a calibrated gripper geometry profile."""

  min_contact_separation_m: float
  max_opening_m: float

  def __post_init__(self) -> None:
    _require_finite_number(
      self.min_contact_separation_m, "min_contact_separation_m"
    )
    _require_finite_number(self.max_opening_m, "max_opening_m")
    if not 0 <= self.min_contact_separation_m <= self.max_opening_m:
      raise ValueError(
        "Require 0 <= min_contact_separation_m <= max_opening_m."
      )


@dataclass(frozen=True)
class ContactPairOptions:
  """Bounded search and geometric normal alignment, not a friction model.

  max_samples bounds attempted surface samples independently of max_pairs.
  max_normal_angle_rad is the maximum angle from either outward normal to its
  outward closing-line direction. Clearance is total extra opening.
  """

  seed: int
  max_pairs: int
  opening_clearance_m: float
  max_samples: int = 2000
  max_normal_angle_rad: float = math.radians(5)

  def __post_init__(self) -> None:
    _require_integer(self.seed, "seed")
    _require_integer(self.max_pairs, "max_pairs", minimum=1)
    _require_finite_number(self.opening_clearance_m, "opening_clearance_m")
    if self.opening_clearance_m < 0:
      raise ValueError("opening_clearance_m must be nonnegative.")
    _require_integer(self.max_samples, "max_samples", minimum=1)
    _require_finite_number(self.max_normal_angle_rad, "max_normal_angle_rad")
    if not 0 <= self.max_normal_angle_rad < math.pi / 2:
      raise ValueError("Require 0 <= max_normal_angle_rad < pi/2.")


@dataclass(frozen=True)
class SurfaceContact:
  """Object-local point, unit outward normal and input triangle provenance.

  Only the generator can verify membership, outward orientation and the index's
  upper bound against its source mesh. The constructor checks numeric structure.
  """

  point: Vec3
  outward_normal: Vec3
  triangle_index: int

  def __post_init__(self) -> None:
    _require_vec3(self.point, "point")
    _require_vec3(self.outward_normal, "outward_normal")
    _require_integer(self.triangle_index, "triangle_index", minimum=0)
    if not math.isclose(
      math.hypot(*self.outward_normal),
      1.0,
      rel_tol=0.0,
      abs_tol=UNIT_NORMAL_TOLERANCE,
    ):
      raise ValueError("outward_normal must have unit length.")


@dataclass(frozen=True)
class ContactPair:
  """Distinct contacts defining a closing line, not an executable grasp.

  Ordering does not assign physical jaw joints. Antipodal alignment and aperture
  compatibility require the mesh, limits and options and are search guarantees.
  """

  first: SurfaceContact
  second: SurfaceContact

  def __post_init__(self) -> None:
    if not isinstance(self.first, SurfaceContact) or not isinstance(
      self.second, SurfaceContact
    ):
      raise ValueError("first and second must be SurfaceContact values.")
    if not math.isfinite(self.separation_m) or self.separation_m <= 0:
      raise ValueError("contact separation must be positive and finite.")

  @property
  def separation_m(self) -> float:
    """Euclidean separation derived from contact points; never stored twice."""
    return math.dist(self.first.point, self.second.point)
