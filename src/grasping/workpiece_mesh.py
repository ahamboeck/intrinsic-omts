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

"""Fetches and validates a workpiece's triangle mesh for grasp planning.

Planners that plan on a mesh get it from here, so they need not know how the
Object World stores it.
"""

import numpy as np
from intrinsic.geometry.proto import geometry_service_pb2
from intrinsic.math.python import data_types, proto_conversion

MAX_TRIANGLES = 10000

# Object World name for the geometry an entity is rendered with.
_VISUAL_GEOMETRY = "Intrinsic_Visual"


def validate_mesh(vertices, triangles):
  """Returns the mesh as (N, 3) float and (M, 3) int arrays, or raises.

  The mesh must be closed and outward facing, with 1 to `MAX_TRIANGLES`
  triangles and no degenerate triangles. Vertices duplicated at seams are
  matched by their coordinates.

  Raises:
    ValueError: If the mesh is invalid.
  """
  vertices = np.asarray(vertices, dtype=float)
  triangles = np.asarray(triangles, dtype=int)
  if vertices.ndim != 2 or vertices.shape[1] != 3:
    raise ValueError("vertices must be a list of [x, y, z] points.")
  if triangles.ndim != 2 or triangles.shape[1] != 3:
    raise ValueError("triangles must be a list of vertex index triples.")
  if not 0 < len(triangles) <= MAX_TRIANGLES:
    raise ValueError(f"Expected 1 to {MAX_TRIANGLES} triangles.")
  if triangles.min() < 0 or triangles.max() >= len(vertices):
    raise ValueError("A triangle references a missing vertex.")
  if not np.isfinite(vertices).all():
    raise ValueError("vertices must be finite.")
  corners = vertices[triangles]
  if np.any(
    np.linalg.norm(
      np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
      axis=1,
    )
    == 0
  ):
    raise ValueError("The mesh contains a degenerate triangle.")
  # GLB seams can store the same point with different vertex indices.
  _, welded = np.unique(vertices, axis=0, return_inverse=True)
  faces = welded[triangles]
  edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
  _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
  # Every edge must occur exactly twice, once in each direction.
  if np.any(counts != 2) or len(np.unique(edges, axis=0)) != len(edges):
    raise ValueError("The mesh must be closed with outward-facing triangles.")
  volume = np.einsum(
    "ij,ij->i", corners[:, 0], np.cross(corners[:, 1], corners[:, 2])
  ).sum()
  if volume <= 0:
    raise ValueError("The mesh must be closed with outward-facing triangles.")
  return vertices, triangles


def center_of_mass(corners):
  """Returns the volume centroid of a closed mesh, its center of mass if solid.

  Sums the signed tetrahedra between each triangle and the origin, so it is
  not biased toward densely meshed regions as the mean of the vertices is.

  Args:
    corners: (T, 3, 3) corners of closed, outward-facing triangles.
  """
  volumes = np.einsum(
    "ij,ij->i", corners[:, 0], np.cross(corners[:, 1], corners[:, 2])
  )
  centroids = corners.sum(axis=1) / 4
  return (volumes[:, None] * centroids).sum(axis=0) / volumes.sum()


def ray_hits(origins, directions, corners):
  """Returns ray to triangle hit distances, inf where a ray misses.

  Uses the Möller-Trumbore test. Memory grows with rays x triangles, so cast
  rays in chunks of a few dozen.

  Args:
    origins: (R, 3) ray origins.
    directions: (R, 3) unit ray directions.
    corners: (T, 3, 3) triangle corners, in the same frame as the rays.

  Returns:
    (R, T) distances along each ray.
  """
  edge1 = corners[:, 1] - corners[:, 0]
  edge2 = corners[:, 2] - corners[:, 0]
  h = np.cross(directions[:, None, :], edge2[None])
  det = np.einsum("tk,rtk->rt", edge1, h)
  with np.errstate(divide="ignore", invalid="ignore"):
    inv = 1.0 / det
    s = origins[:, None, :] - corners[None, :, 0]
    u = np.einsum("rtk,rtk->rt", s, h) * inv
    q = np.cross(s, edge1[None])
    v = np.einsum("rk,rtk->rt", directions, q) * inv
    t = np.einsum("tk,rtk->rt", edge2, q) * inv
    hit = (
      (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    )
  return np.where(hit, t, np.inf)


def _matrix(transform):
  """Returns a GeometricTransform as a 4x4 matrix."""
  if transform.HasField("matrix4d"):
    return proto_conversion.ndarray_from_matrix_proto(transform.matrix4d)
  matrix = np.eye(4)
  if transform.HasField("trs"):
    trs = transform.trs
    rpy = trs.rotation_rpy
    matrix[:3, :3] = data_types.Rotation3.from_euler_angles(
      rpy_radians=[rpy.r, rpy.p, rpy.y]
    ).matrix3x3()
    if trs.HasField("scale"):
      matrix[:3, :3] *= [trs.scale.x, trs.scale.y, trs.scale.z]
    matrix[:3, 3] = [trs.translation.x, trs.translation.y, trs.translation.z]
  return matrix


def workpiece_mesh(workpiece, geometry_service):
  """Returns the validated visual mesh of an Object World object.

  Args:
    workpiece: Object World object, as returned by `get_object`.
    geometry_service: Geometry service stub, used for meshes stored by
      reference.

  Returns:
    (N, 3) vertices in the object's frame and (M, 3) triangle indices.

  Raises:
    ValueError: If the object has no visual mesh, or the mesh is invalid.
  """
  proto = workpiece.proto
  vertices, triangles = [], []
  for entity in proto.entities.values():
    named = entity.geometry_component.named_geometries
    if _VISUAL_GEOMETRY not in named:
      continue
    object_t_entity = np.eye(4)
    link = entity
    while link.id != proto.root_entity_id:
      parent_t_link = proto_conversion.pose_from_proto(link.parent_t_this)
      object_t_entity = parent_t_link.matrix4x4() @ object_t_entity
      link = proto.entities[link.parent_id]
    for shape in named[_VISUAL_GEOMETRY].named_geometries.values():
      inline = shape.geometry.inline_geometry_data
      if shape.geometry.HasField("geo_ref"):
        inline = geometry_service.GetGeometry(
          geometry_service_pb2.GetGeometryRequest(
            geometry_storage_refs=shape.geometry.geo_ref
          )
        ).inline_geometry
      if not inline.exact_geometry.HasField("triangle_mesh"):
        raise ValueError(
          f"{proto.name} has a visual geometry that is not a mesh."
        )
      mesh = inline.exact_geometry.triangle_mesh
      object_t_shape = object_t_entity @ _matrix(shape.ref_t_shape)
      points = np.reshape(mesh.vertices, (-1, 3))
      triangles.append(
        np.reshape(mesh.faces, (-1, 3)) + sum(map(len, vertices))
      )
      vertices.append(points @ object_t_shape[:3, :3].T + object_t_shape[:3, 3])
  if not vertices:
    raise ValueError(f"{proto.name} has no visual mesh.")
  return validate_mesh(np.concatenate(vertices), np.concatenate(triangles))
