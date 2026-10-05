"""Contract tests for the pure, object-local grasping inputs and outputs."""

import dataclasses
import math
import unittest

from src.grasping import contact_pairs, types

VERTICES = ((0.0, 0.0, 0.0), (0.04, 0.0, 0.0), (0.0, 0.03, 0.0))
TRIANGLES = ((0, 1, 2),)


class GraspingTypesTest(unittest.TestCase):
  def test_valid_mesh_preserves_object_local_geometry(self):
    mesh = types.TriangleMesh(VERTICES, TRIANGLES)
    self.assertEqual(mesh.vertices, VERTICES)
    self.assertEqual(mesh.triangles, TRIANGLES)

  def test_mesh_requires_nonempty_tuple_storage(self):
    for vertices, triangles in (
      ((), TRIANGLES),
      (VERTICES, ()),
      (list(VERTICES), TRIANGLES),
      ((list(VERTICES[0]), *VERTICES[1:]), TRIANGLES),
      (VERTICES, list(TRIANGLES)),
      (VERTICES, ([0, 1, 2],)),
      (None, TRIANGLES),
    ):
      with self.subTest(vertices=vertices, triangles=triangles):
        with self.assertRaises(ValueError):
          types.TriangleMesh(vertices, triangles)

  def test_mesh_rejects_malformed_vertices(self):
    for vertex in ((0.0, 0.0), (0.0,) * 4, (True, 0, 0), ("0", 0, 0)):
      with self.subTest(vertex=vertex):
        with self.assertRaises(ValueError):
          types.TriangleMesh((vertex, *VERTICES[1:]), TRIANGLES)

  def test_nonfinite_coordinates_rejected_in_mesh_and_contact(self):
    for value in (math.nan, math.inf, -math.inf):
      with self.subTest(value=value):
        with self.assertRaises(ValueError):
          types.TriangleMesh(((value, 0, 0), *VERTICES[1:]), TRIANGLES)
        with self.assertRaises(ValueError):
          types.SurfaceContact((value, 0, 0), (1, 0, 0), 0)
        with self.assertRaises(ValueError):
          types.SurfaceContact((0, 0, 0), (value, 0, 0), 0)

  def test_mesh_rejects_malformed_triangle_indices(self):
    for triangle in (
      (0, 1),
      (0, 1, 2, 0),
      (-1, 1, 2),
      (0, 1, 3),
      (0, 1, 2.0),
      (False, 1, 2),
      (0, 1, "2"),
    ):
      with self.subTest(triangle=triangle):
        with self.assertRaises(ValueError):
          types.TriangleMesh(VERTICES, (triangle,))

  def test_mesh_rejects_degenerate_triangles(self):
    for vertices, triangles in (
      (VERTICES, ((0, 0, 2),)),
      (((0, 0, 0), (1, 1, 1), (2, 2, 2)), TRIANGLES),
      (((0, 0, 0), (0, 0, 0), (0, 1, 0)), TRIANGLES),
    ):
      with self.subTest(vertices=vertices, triangles=triangles):
        with self.assertRaises(ValueError):
          types.TriangleMesh(vertices, triangles)

  def test_small_nondegenerate_triangle_is_not_rejected_by_area_threshold(self):
    mesh = types.TriangleMesh(
      ((0, 0, 0), (1e-100, 0, 0), (0, 1e-100, 0)), TRIANGLES
    )
    self.assertEqual(len(mesh.triangles), 1)

  def test_mesh_rejects_unrepresentable_edge_lengths(self):
    with self.assertRaises(ValueError):
      types.TriangleMesh(((-1e308, 0, 0), (1e308, 0, 0), (0, 1, 0)), TRIANGLES)

  def test_unrepresentable_integers_raise_value_error(self):
    too_large = 10**1000
    with self.assertRaises(ValueError):
      types.ParallelJawLimits(0, too_large)
    with self.assertRaises(ValueError):
      types.SurfaceContact((too_large, 0, 0), (1, 0, 0), 0)

  def test_jaw_limit_boundaries(self):
    for minimum, maximum in ((0, 0), (0, 0.05), (0.05, 0.05)):
      with self.subTest(minimum=minimum, maximum=maximum):
        limits = types.ParallelJawLimits(minimum, maximum)
        self.assertEqual(limits.max_opening_m, maximum)

  def test_invalid_jaw_limits(self):
    for minimum, maximum in (
      (-0.01, 0.05),
      (0.06, 0.05),
      (0, -1),
      (math.nan, 0.05),
      (0, math.inf),
      (True, 1),
      (0, "0.05"),
    ):
      with self.subTest(minimum=minimum, maximum=maximum):
        with self.assertRaises(ValueError):
          types.ParallelJawLimits(minimum, maximum)

  def test_valid_options_allow_negative_seed_and_zero_clearance(self):
    options = types.ContactPairOptions(
      seed=-5, max_pairs=1, opening_clearance_m=0
    )
    self.assertEqual(options.seed, -5)
    self.assertEqual(options.max_pairs, 1)
    self.assertEqual(options.opening_clearance_m, 0)

  def test_invalid_option_integer_fields(self):
    for seed in (True, 1.5, "1", None):
      with self.subTest(seed=seed):
        with self.assertRaises(ValueError):
          types.ContactPairOptions(seed, 20, 0.001)
    for budget in (0, -1, True, 1.5, "20", None):
      with self.subTest(budget=budget):
        with self.assertRaises(ValueError):
          types.ContactPairOptions(0, budget, 0.001)

  def test_invalid_opening_clearance(self):
    for clearance in (-0.001, math.nan, math.inf, -math.inf, True, "0.001"):
      with self.subTest(clearance=clearance):
        with self.assertRaises(ValueError):
          types.ContactPairOptions(0, 20, clearance)

  def test_contact_retains_provenance_without_claiming_mesh_membership(self):
    contact = types.SurfaceContact((0, 0, 0), (-1, 0, 0), 100)
    self.assertEqual(contact.triangle_index, 100)
    self.assertEqual(contact.outward_normal, (-1, 0, 0))

  def test_contact_rejects_invalid_normal(self):
    for normal in ((0, 0, 0), (2, 0, 0), (1, 1, 0), (1, 0), [1, 0, 0]):
      with self.subTest(normal=normal):
        with self.assertRaises(ValueError):
          types.SurfaceContact((0, 0, 0), normal, 0)

  def test_unit_normal_tolerance_is_absolute_on_length(self):
    tolerance = types.UNIT_NORMAL_TOLERANCE
    types.SurfaceContact((0, 0, 0), (1 + tolerance / 2, 0, 0), 0)
    with self.assertRaises(ValueError):
      types.SurfaceContact((0, 0, 0), (1 + tolerance * 2, 0, 0), 0)

  def test_contact_rejects_invalid_point_or_index(self):
    for point in ((0, 0), [0, 0, 0], (False, 0, 0)):
      with self.subTest(point=point):
        with self.assertRaises(ValueError):
          types.SurfaceContact(point, (1, 0, 0), 0)
    for index in (-1, True, 0.5, "0"):
      with self.subTest(index=index):
        with self.assertRaises(ValueError):
          types.SurfaceContact((0, 0, 0), (1, 0, 0), index)

  def test_pair_separation_is_derived_and_independent_of_order(self):
    first = types.SurfaceContact((0, 0, 0), (-1, 0, 0), 0)
    second = types.SurfaceContact((0.03, 0.04, 0), (1, 0, 0), 1)
    self.assertAlmostEqual(types.ContactPair(first, second).separation_m, 0.05)
    self.assertAlmostEqual(types.ContactPair(second, first).separation_m, 0.05)

  def test_pair_requires_distinct_contacts_and_finite_separation(self):
    first = types.SurfaceContact((0, 0, 0), (1, 0, 0), 0)
    same_point = types.SurfaceContact((0, 0, 0), (-1, 0, 0), 1)
    for second in (same_point, None):
      with self.subTest(second=second):
        with self.assertRaises(ValueError):
          types.ContactPair(first, second)
    with self.assertRaises(ValueError):
      types.ContactPair(
        types.SurfaceContact((-1e308, 0, 0), (-1, 0, 0), 0),
        types.SurfaceContact((1e308, 0, 0), (1, 0, 0), 1),
      )

  def test_contracts_are_frozen_and_nested_geometry_is_immutable(self):
    mesh = types.TriangleMesh(VERTICES, TRIANGLES)
    contact = types.SurfaceContact((0, 0, 0), (1, 0, 0), 0)
    pair = types.ContactPair(
      contact, types.SurfaceContact((0.01, 0, 0), (-1, 0, 0), 1)
    )
    objects_and_fields = (
      (mesh, "vertices"),
      (contact, "point"),
      (pair, "first"),
      (types.ParallelJawLimits(0, 0.05), "max_opening_m"),
      (types.ContactPairOptions(0, 20, 0.001), "seed"),
    )
    for obj, field in objects_and_fields:
      with self.subTest(obj=obj):
        with self.assertRaises(dataclasses.FrozenInstanceError):
          setattr(obj, field, None)
    with self.assertRaises(TypeError):
      mesh.vertices[0][0] = 1
    with self.assertRaises(TypeError):
      mesh.triangles[0][0] = 1

  def test_search_placeholder_does_not_claim_empty_search_results(self):
    with self.assertRaisesRegex(
      NotImplementedError, "Contact-pair search is not implemented yet\\."
    ):
      contact_pairs.propose_contact_pairs(
        types.TriangleMesh(VERTICES, TRIANGLES),
        types.ParallelJawLimits(0, 0.05),
        options=types.ContactPairOptions(0, 20, 0.001),
      )


if __name__ == "__main__":
  unittest.main()
