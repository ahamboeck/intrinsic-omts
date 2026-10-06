"""Bounded geometric antipodal search on a closed outward-wound triangle shell.

Self-intersections are not detected. Inputs must be embedded (nonintersecting)
surfaces. Returned pairs are geometry proposals, not validated gripper poses.
"""

import bisect
import math
import random
from collections import defaultdict
from dataclasses import dataclass

from src.grasping.types import (
  ContactPair,
  ContactPairOptions,
  ParallelJawLimits,
  SurfaceContact,
  TriangleMesh,
  Vec3,
)

# Computation uses a mesh-diagonal-normalized local frame. Ambiguous edge,
# vertex, near-origin and tied ray hits are rejected at this resolution.
_GEOMETRY_EPS = 1e-10
_DUPLICATE_DISTANCE = 1e-6  # Fraction of the mesh's bounding-box diagonal.
_ALIGNMENT_EPS = 1e-12  # Dot-product roundoff allowance, not extra cone angle.


def _sub(a: Vec3, b: Vec3) -> Vec3:
  return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Vec3, b: Vec3) -> float:
  return math.fsum(x * y for x, y in zip(a, b, strict=True))


def _cross(a: Vec3, b: Vec3) -> Vec3:
  return (
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  )


def _connected(
  start: int, allowed: set[int], neighbors: list[set[int]]
) -> bool:
  reached, pending = {start}, [start]
  while pending:
    current = pending.pop()
    for adjacent in (neighbors[current] & allowed) - reached:
      reached.add(adjacent)
      pending.append(adjacent)
  return reached == allowed


def _validate_topology(mesh: TriangleMesh) -> None:
  edges = defaultdict(list)
  incident = defaultdict(set)
  faces = set()
  for index, triangle in enumerate(mesh.triangles):
    key = tuple(sorted(triangle))
    if key in faces:
      raise ValueError("Duplicate mesh faces are unsupported.")
    faces.add(key)
    for a, b in zip(triangle, (*triangle[1:], triangle[0]), strict=True):
      edges[min(a, b), max(a, b)].append((index, a, b))
      incident[a].add(index)
  neighbors = [set() for _ in mesh.triangles]
  for uses in edges.values():
    if len(uses) != 2:
      raise ValueError(
        "Require a closed two-manifold shell: every edge needs two faces."
      )
    (first, a, b), (second, c, d) = uses
    if (a, b) != (d, c):
      raise ValueError("Mesh faces must have consistent winding.")
    neighbors[first].add(second)
    neighbors[second].add(first)
  if not _connected(0, set(range(len(mesh.triangles))), neighbors):
    raise ValueError("Only one connected shell is supported.")
  for vertex_faces in incident.values():
    if not _connected(next(iter(vertex_faces)), vertex_faces, neighbors):
      raise ValueError("Nonmanifold mesh vertices are unsupported.")


@dataclass(frozen=True)
class _Face:
  origin: Vec3
  edge1: Vec3
  edge2: Vec3
  normal: Vec3
  double_area: float


def _prepare(mesh: TriangleMesh) -> tuple[Vec3, float, list[_Face]]:
  _validate_topology(mesh)
  used = {i for triangle in mesh.triangles for i in triangle}
  origin = mesh.vertices[mesh.triangles[0][0]]
  extents = tuple(
    max(mesh.vertices[i][axis] for i in used)
    - min(mesh.vertices[i][axis] for i in used)
    for axis in range(3)
  )
  scale = math.hypot(*extents)
  if not math.isfinite(scale) or scale <= 0:
    raise ValueError("Mesh extent must be positive and finite.")
  vertices = tuple(
    tuple(value / scale for value in _sub(vertex, origin))
    for vertex in mesh.vertices
  )
  faces, volumes = [], []
  for triangle in mesh.triangles:
    a, b, c = (vertices[i] for i in triangle)
    edge1, edge2 = _sub(b, a), _sub(c, a)
    normal = _cross(edge1, edge2)
    double_area = math.hypot(*normal)
    if double_area <= _GEOMETRY_EPS * math.hypot(*edge1) * math.hypot(*edge2):
      raise ValueError(
        "Mesh contains faces below the angular numeric resolution."
      )
    faces.append(
      _Face(
        a, edge1, edge2, tuple(v / double_area for v in normal), double_area
      )
    )
    volumes.append(_dot(a, _cross(b, c)) / 6)
  if math.fsum(volumes) <= _GEOMETRY_EPS:
    raise ValueError(
      "Require an outward-wound shell with positive resolvable volume."
    )
  return origin, scale, faces


def _ray_hit(
  point: Vec3, direction: Vec3, face: _Face
) -> tuple[float, bool] | None:
  """Two-sided Moller-Trumbore hit; boundary ambiguity must not be skipped."""
  p = _cross(direction, face.edge2)
  determinant = _dot(face.edge1, p)
  if abs(determinant) <= _GEOMETRY_EPS * face.double_area:
    return None
  offset = _sub(point, face.origin)
  u = _dot(offset, p) / determinant
  q = _cross(offset, face.edge1)
  v = _dot(direction, q) / determinant
  distance = _dot(face.edge2, q) / determinant
  if (
    u < -_GEOMETRY_EPS
    or v < -_GEOMETRY_EPS
    or u + v > 1 + _GEOMETRY_EPS
    or distance < -_GEOMETRY_EPS
  ):
    return None
  boundary = min(u, v, 1 - u - v) <= _GEOMETRY_EPS or distance <= _GEOMETRY_EPS
  return distance, boundary


def _first_exit(
  point: Vec3, direction: Vec3, source: int, faces: list[_Face]
) -> tuple[int, float] | None:
  nearest, index, ambiguous = math.inf, -1, False
  for other, face in enumerate(faces):
    if other == source:
      continue
    hit = _ray_hit(point, direction, face)
    if hit is None:
      continue
    distance, boundary = hit
    if distance < nearest - _GEOMETRY_EPS:
      nearest, index, ambiguous = distance, other, boundary
    elif abs(distance - nearest) <= _GEOMETRY_EPS:
      ambiguous = True
  if index < 0 or ambiguous:
    return None
  return index, nearest


def _duplicate(
  first: Vec3, second: Vec3, accepted: list[tuple[Vec3, Vec3]]
) -> bool:
  return any(
    (
      math.dist(first, a) <= _DUPLICATE_DISTANCE
      and math.dist(second, b) <= _DUPLICATE_DISTANCE
    )
    or (
      math.dist(first, b) <= _DUPLICATE_DISTANCE
      and math.dist(second, a) <= _DUPLICATE_DISTANCE
    )
    for a, b in accepted
  )


def propose_contact_pairs(
  mesh: TriangleMesh,
  gripper: ParallelJawLimits,
  *,
  options: ContactPairOptions,
) -> list[ContactPair]:
  """Return bounded contact proposals using area-uniform inward normal rays.

  Invalid/unsupported topology raises ValueError. An empty list means a finite
  search found no acceptable pairs, not that no physical grasp exists. The first
  exit, never a farther surface beyond a void, supplies the second contact.
  """
  origin, scale, faces = _prepare(mesh)
  cumulative, total = [], 0.0
  for face in faces:
    total += face.double_area
    cumulative.append(total)
  rng = random.Random(options.seed)
  cosine = math.cos(options.max_normal_angle_rad)
  pairs, accepted = [], []

  def object_point(point: Vec3) -> Vec3:
    return tuple(a + scale * b for a, b in zip(origin, point, strict=True))

  for _ in range(options.max_samples):
    source = min(
      bisect.bisect_right(cumulative, rng.random() * total), len(faces) - 1
    )
    face = faces[source]
    root, fraction = math.sqrt(rng.random()), rng.random()
    u, v = root * (1 - fraction), root * fraction
    if min(u, v, 1 - u - v) <= _GEOMETRY_EPS:
      continue
    first = tuple(
      a + u * b + v * c
      for a, b, c in zip(face.origin, face.edge1, face.edge2, strict=True)
    )
    direction = tuple(-component for component in face.normal)
    hit = _first_exit(first, direction, source, faces)
    if hit is None:
      continue
    target, distance = hit
    if _dot(direction, faces[target].normal) + _ALIGNMENT_EPS < cosine:
      continue
    second = tuple(
      a + distance * b for a, b in zip(first, direction, strict=True)
    )
    if _duplicate(first, second, accepted):
      continue
    pair = ContactPair(
      SurfaceContact(object_point(first), face.normal, source),
      SurfaceContact(object_point(second), faces[target].normal, target),
    )
    # Recheck in the returned coordinate frame, not just normalized ray units.
    if (
      pair.separation_m < gripper.min_contact_separation_m
      or pair.separation_m + options.opening_clearance_m > gripper.max_opening_m
    ):
      continue
    closing = tuple(
      v / pair.separation_m for v in _sub(pair.second.point, pair.first.point)
    )
    if (
      min(_dot(closing, direction), _dot(closing, faces[target].normal))
      + _ALIGNMENT_EPS
      < cosine
    ):
      continue
    pairs.append(pair)
    accepted.append((first, second))
    if len(pairs) == options.max_pairs:
      break
  return pairs
