"""Explicit numeric geometry conversion, not asset loading or a box fallback."""

from src.grasping.geometry import RigidTransform, scale
from src.grasping.types import (
  TriangleMesh,
  Vec3,
  _require_finite_number,
  _require_vec3,
)


def normalize_mesh(
  mesh: TriangleMesh,
  *,
  metres_per_unit: float,
  object_T_mesh: RigidTransform,
) -> TriangleMesh:
  """Scale asset vertices to metres, then map them into the object frame.

  The transform's translation is already in metres. Positive uniform scaling
  preserves winding; reflections, topology repair and bounding-box replacement
  are deliberately unsupported. The caller must establish asset provenance.
  """
  _require_finite_number(metres_per_unit, "metres_per_unit")
  if metres_per_unit <= 0:
    raise ValueError("metres_per_unit must be positive.")
  if not isinstance(mesh, TriangleMesh) or not isinstance(
    object_T_mesh, RigidTransform
  ):
    raise ValueError("require a triangle mesh and object_T_mesh transform.")
  return TriangleMesh(
    tuple(
      object_T_mesh.apply(scale(v, metres_per_unit)) for v in mesh.vertices
    ),
    mesh.triangles,
  )


def cuboid_mesh(dimensions_m: Vec3) -> TriangleMesh:
  """Exactly triangulate a centered cuboid primitive with FULL XYZ dimensions.

  Only use for an actual cuboid primitive, never the bounding box of a mesh.
  Entity/asset transforms must still be applied with normalize_mesh.
  """
  _require_vec3(dimensions_m, "dimensions_m")
  if any(d <= 0 for d in dimensions_m):
    raise ValueError("cuboid dimensions must be positive.")
  x, y, z = (d / 2 for d in dimensions_m)
  return TriangleMesh(
    (
      (-x, -y, -z),
      (x, -y, -z),
      (x, y, -z),
      (-x, y, -z),
      (-x, -y, z),
      (x, -y, z),
      (x, y, z),
      (-x, y, z),
    ),
    (
      (0, 2, 1),
      (0, 3, 2),
      (4, 5, 6),
      (4, 6, 7),
      (0, 1, 5),
      (0, 5, 4),
      (1, 2, 6),
      (1, 6, 5),
      (2, 3, 7),
      (2, 7, 6),
      (3, 0, 4),
      (3, 4, 7),
    ),
  )
