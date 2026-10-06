"""Mesh-search contracts tested on analytic closed surfaces, without a robot."""

import math
import random
import unittest
from unittest import mock

from src.grasping.contact_pairs import propose_contact_pairs
from src.grasping.types import (
  ContactPairOptions,
  ParallelJawLimits,
  TriangleMesh,
)


def extrude(polygon, planar_triangles, height):
  """Extrude a CCW polygon; use an explicit interior triangulation."""
  count = len(polygon)
  vertices = tuple((x, y, z) for z in (0, height) for x, y in polygon)
  triangles = [(c, b, a) for a, b, c in planar_triangles]
  triangles += [
    (a + count, b + count, c + count) for a, b, c in planar_triangles
  ]
  for a in range(count):
    b = (a + 1) % count
    triangles.extend(((a, b, b + count), (a, b + count, a + count)))
  return TriangleMesh(vertices, tuple(triangles))


def box(x=0.04, y=0.03, z=0.02):
  return extrude(((0, 0), (x, 0), (x, y), (0, y)), ((0, 1, 2), (0, 2, 3)), z)


def l_profile():
  return extrude(
    ((0, 0), (0.06, 0), (0.06, 0.02), (0.02, 0.02), (0.02, 0.06), (0, 0.06)),
    ((0, 1, 3), (1, 2, 3), (0, 3, 5), (3, 4, 5)),
    0.015,
  )


def u_profile():
  return extrude(
    (
      (0, 0),
      (0.06, 0),
      (0.06, 0.06),
      (0.04, 0.06),
      (0.04, 0.02),
      (0.02, 0.02),
      (0.02, 0.06),
      (0, 0.06),
    ),
    ((0, 1, 4), (0, 4, 5), (1, 2, 3), (1, 3, 4), (0, 5, 6), (0, 6, 7)),
    0.015,
  )


def sub(a, b):
  return tuple(x - y for x, y in zip(a, b, strict=True))


def dot(a, b):
  return sum(x * y for x, y in zip(a, b, strict=True))


def cross(a, b):
  return (
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  )


class ContactPairsTest(unittest.TestCase):
  def assert_surface_contact(self, mesh, contact):
    self.assertGreaterEqual(contact.triangle_index, 0)
    self.assertLess(contact.triangle_index, len(mesh.triangles))
    a, b, c = (mesh.vertices[i] for i in mesh.triangles[contact.triangle_index])
    ab, ac, ap = sub(b, a), sub(c, a), sub(contact.point, a)
    normal = cross(ab, ac)
    length = math.hypot(*normal)
    self.assertAlmostEqual(dot(ap, normal) / length, 0, delta=1e-10)
    # Independent barycentric surface-membership check, not the ray algorithm.
    denominator = dot(ab, ab) * dot(ac, ac) - dot(ab, ac) ** 2
    v = (dot(ac, ac) * dot(ap, ab) - dot(ab, ac) * dot(ap, ac)) / denominator
    w = (dot(ab, ab) * dot(ap, ac) - dot(ab, ac) * dot(ap, ab)) / denominator
    self.assertGreaterEqual(v, -1e-9)
    self.assertGreaterEqual(w, -1e-9)
    self.assertLessEqual(v + w, 1 + 1e-9)
    for actual, expected in zip(contact.outward_normal, normal, strict=True):
      self.assertAlmostEqual(actual, expected / length, places=10)

  def assert_pairs_valid(self, mesh, pairs, limits, options):
    self.assertLessEqual(len(pairs), options.max_pairs)
    self.assertLessEqual(len(pairs), options.max_samples)
    for pair in pairs:
      self.assert_surface_contact(mesh, pair.first)
      self.assert_surface_contact(mesh, pair.second)
      direction = tuple(
        v / pair.separation_m for v in sub(pair.second.point, pair.first.point)
      )
      self.assertGreaterEqual(
        dot(direction, tuple(-v for v in pair.first.outward_normal)),
        math.cos(options.max_normal_angle_rad) - 1e-10,
      )
      self.assertGreaterEqual(
        dot(direction, pair.second.outward_normal),
        math.cos(options.max_normal_angle_rad) - 1e-10,
      )
      self.assertGreaterEqual(
        pair.separation_m, limits.min_contact_separation_m
      )
      self.assertLessEqual(
        pair.separation_m + options.opening_clearance_m, limits.max_opening_m
      )

  def test_box_contacts_have_real_surface_provenance_and_opposed_normals(self):
    mesh, limits, options = (
      box(),
      ParallelJawLimits(0, 0.05),
      ContactPairOptions(42, 20, 0.001),
    )
    pairs = propose_contact_pairs(mesh, limits, options=options)
    self.assertEqual(len(pairs), 20)
    self.assert_pairs_valid(mesh, pairs, limits, options)
    for pair in pairs:
      self.assertTrue(
        any(
          math.isclose(pair.separation_m, width) for width in (0.02, 0.03, 0.04)
        )
      )

  def test_concave_profiles_do_not_cross_empty_notches(self):
    limits, options = ParallelJawLimits(0, 0.1), ContactPairOptions(7, 200, 0)
    for mesh, in_solid in (
      (l_profile(), lambda x, y: x <= 0.02 + 1e-10 or y <= 0.02 + 1e-10),
      (
        u_profile(),
        lambda x, y: x <= 0.02 + 1e-10
        or x >= 0.04 - 1e-10
        or y <= 0.02 + 1e-10,
      ),
    ):
      with self.subTest(mesh=mesh):
        pairs = propose_contact_pairs(mesh, limits, options=options)
        self.assertGreater(len(pairs), 0)
        self.assert_pairs_valid(mesh, pairs, limits, options)
        # Analytic footprint also checks the interior of the closing segment.
        for pair in pairs:
          for step in range(21):
            t = step / 20
            p = tuple(
              (1 - t) * a + t * b
              for a, b in zip(pair.first.point, pair.second.point, strict=True)
            )
            self.assertTrue(in_solid(p[0], p[1]), p)

  def test_too_wide_or_too_small_for_minimum_returns_empty(self):
    for limits in (ParallelJawLimits(0, 0.01), ParallelJawLimits(0.045, 0.05)):
      with self.subTest(limits=limits):
        self.assertEqual(
          propose_contact_pairs(
            box(), limits, options=ContactPairOptions(0, 20, 0, max_samples=50)
          ),
          [],
        )

  def test_total_clearance_and_aperture_boundaries(self):
    mesh = box(0.04, 0.04, 0.04)
    limits = ParallelJawLimits(0.04 - 1e-12, 0.045)
    options = ContactPairOptions(5, 20, 0.004)
    pairs = propose_contact_pairs(mesh, limits, options=options)
    self.assertGreater(
      len(pairs), 0
    )  # Would fail if clearance were per finger.
    self.assert_pairs_valid(mesh, pairs, limits, options)
    self.assertEqual(
      propose_contact_pairs(
        mesh, limits, options=ContactPairOptions(5, 20, 0.006)
      ),
      [],
    )

  def test_seed_reproducibility_and_no_global_random_state_changes(self):
    mesh, limits, options = (
      box(),
      ParallelJawLimits(0, 0.1),
      ContactPairOptions(-3, 20, 0),
    )
    state = random.getstate()
    first = propose_contact_pairs(mesh, limits, options=options)
    self.assertEqual(
      first, propose_contact_pairs(mesh, limits, options=options)
    )
    self.assertEqual(random.getstate(), state)
    self.assertNotEqual(
      first,
      propose_contact_pairs(mesh, limits, options=ContactPairOptions(4, 20, 0)),
    )

  def test_exact_opening_and_minimum_boundaries_use_returned_separation(self):
    mesh = box()
    options = ContactPairOptions(0, 1, 0.001, max_samples=1)
    pair = propose_contact_pairs(
      mesh, ParallelJawLimits(0, 0.1), options=options
    )[0]
    maximum = pair.separation_m + options.opening_clearance_m
    self.assertEqual(
      propose_contact_pairs(
        mesh, ParallelJawLimits(pair.separation_m, maximum), options=options
      ),
      [pair],
    )
    self.assertEqual(
      propose_contact_pairs(
        mesh, ParallelJawLimits(0, math.nextafter(maximum, 0)), options=options
      ),
      [],
    )
    self.assertEqual(
      propose_contact_pairs(
        mesh,
        ParallelJawLimits(math.nextafter(pair.separation_m, math.inf), 0.1),
        options=options,
      ),
      [],
    )

  def test_zero_volume_shell_is_rejected(self):
    mesh = TriangleMesh(
      ((0, 0, 0), (0.04, 0, 0), (0, 0.04, 0), (0.04, 0.04, 0)),
      ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)),
    )
    with self.assertRaisesRegex(ValueError, "volume"):
      propose_contact_pairs(
        mesh, ParallelJawLimits(0, 0.1), options=ContactPairOptions(0, 20, 0)
      )

  def test_sample_budget_is_independent_of_output_budget(self):
    options = ContactPairOptions(0, 100, 0, max_samples=3)
    pairs = propose_contact_pairs(
      box(), ParallelJawLimits(0, 0.1), options=options
    )
    self.assertGreater(len(pairs), 0)
    self.assertLessEqual(len(pairs), 3)

  def test_repeated_samples_do_not_duplicate_contact_pairs(self):
    rng = mock.Mock()
    rng.random.return_value = 0.4
    with mock.patch(
      "src.grasping.contact_pairs.random.Random", return_value=rng
    ):
      pairs = propose_contact_pairs(
        box(),
        ParallelJawLimits(0, 0.1),
        options=ContactPairOptions(0, 20, 0, max_samples=5),
      )
    self.assertEqual(len(pairs), 1)

  def test_source_edge_and_vertex_samples_are_conservatively_skipped(self):
    rng = mock.Mock()
    rng.random.return_value = 0
    with mock.patch(
      "src.grasping.contact_pairs.random.Random", return_value=rng
    ):
      pairs = propose_contact_pairs(
        box(),
        ParallelJawLimits(0, 0.1),
        options=ContactPairOptions(0, 20, 0, max_samples=5),
      )
    self.assertEqual(pairs, [])

  def test_sampling_is_area_weighted_not_uniform_over_triangles(self):
    mesh = box(0.08, 0.02, 0.01)
    pairs = propose_contact_pairs(
      mesh, ParallelJawLimits(0, 0.1), options=ContactPairOptions(8, 600, 0)
    )
    z_faces = sum(abs(pair.first.outward_normal[2]) > 0.9 for pair in pairs)
    # z-face area is 8/13 of total area; equal face/triangle sampling is 1/3.
    self.assertGreater(z_faces / len(pairs), 0.53)
    self.assertLess(z_faces / len(pairs), 0.70)

  def test_rigid_transform_preserves_pairs_and_normals(self):
    mesh, limits, options = (
      box(),
      ParallelJawLimits(0, 0.1),
      ContactPairOptions(9, 20, 0),
    )

    def rotate(p):
      return (-p[1], p[2], -p[0])

    def transform(p):
      return tuple(
        v + offset
        for v, offset in zip(rotate(p), (0.7, -0.3, 1.2), strict=True)
      )

    moved = TriangleMesh(
      tuple(transform(p) for p in mesh.vertices), mesh.triangles
    )
    original = propose_contact_pairs(mesh, limits, options=options)
    transformed = propose_contact_pairs(moved, limits, options=options)
    self.assertEqual(len(original), len(transformed))
    for first, second in zip(original, transformed, strict=True):
      for a, b in ((first.first, second.first), (first.second, second.second)):
        self.assertLess(math.dist(transform(a.point), b.point), 1e-10)
        self.assertLess(
          math.dist(rotate(a.outward_normal), b.outward_normal), 1e-10
        )

  def test_rejects_open_inconsistent_inward_and_duplicate_shell_faces(self):
    mesh = box()
    triangles = mesh.triangles
    for bad in (
      triangles[:-1],
      (triangles[0][::-1], *triangles[1:]),
      tuple(t[::-1] for t in triangles),
      (*triangles, triangles[0]),
    ):
      with self.subTest(triangles=bad):
        with self.assertRaises(ValueError):
          propose_contact_pairs(
            TriangleMesh(mesh.vertices, bad),
            ParallelJawLimits(0, 0.1),
            options=ContactPairOptions(0, 20, 0),
          )

  def test_rejects_disconnected_shells(self):
    mesh = box()
    second = tuple((x + 0.1, y, z) for x, y, z in mesh.vertices)
    triangles = mesh.triangles + tuple(
      tuple(i + len(mesh.vertices) for i in t) for t in mesh.triangles
    )
    with self.assertRaises(ValueError):
      propose_contact_pairs(
        TriangleMesh(mesh.vertices + second, triangles),
        ParallelJawLimits(0, 0.1),
        options=ContactPairOptions(0, 20, 0),
      )

  def test_rejects_pinched_nonmanifold_vertex(self):
    mesh = box()
    # Subdivide so opposite corners have no shared neighbors, then identify
    # them. Edge closure still holds, but their vertex fans remain disjoint.
    vertices, midpoints, triangles = list(mesh.vertices), {}, []
    for a, b, c in mesh.triangles:
      mids = []
      for first, second in ((a, b), (b, c), (c, a)):
        key = tuple(sorted((first, second)))
        if key not in midpoints:
          midpoints[key] = len(vertices)
          vertices.append(
            tuple(
              (x + y) / 2
              for x, y in zip(vertices[first], vertices[second], strict=True)
            )
          )
        mids.append(midpoints[key])
      ab, bc, ca = mids
      triangles.extend(((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)))
    triangles = tuple(tuple(0 if i == 6 else i for i in t) for t in triangles)
    pinched = TriangleMesh(tuple(vertices), triangles)
    with self.assertRaisesRegex(ValueError, "Nonmanifold"):
      propose_contact_pairs(
        pinched, ParallelJawLimits(0, 0.1), options=ContactPairOptions(0, 20, 0)
      )

  def test_uniform_mesh_scale_preserves_geometry(self):
    mesh = box()
    options = ContactPairOptions(12, 20, 0)
    original = propose_contact_pairs(
      mesh, ParallelJawLimits(0, 0.1), options=options
    )
    for scale in (1e-9, 1e9):
      with self.subTest(scale=scale):
        scaled = TriangleMesh(
          tuple(
            tuple(value * scale for value in point) for point in mesh.vertices
          ),
          mesh.triangles,
        )
        pairs = propose_contact_pairs(
          scaled, ParallelJawLimits(0, 0.1 * scale), options=options
        )
        self.assertEqual(len(pairs), len(original))
        for a, b in zip(original, pairs, strict=True):
          self.assertAlmostEqual(
            b.separation_m / scale, a.separation_m, places=10
          )
          for first, second in ((a.first, b.first), (a.second, b.second)):
            self.assertLess(
              math.dist(
                first.point, tuple(value / scale for value in second.point)
              ),
              1e-10,
            )

  def test_angular_alignment_is_checked_against_closing_line(self):
    # Tetrahedral face normals are not aligned with the inward ray at its exit.
    mesh = TriangleMesh(
      ((0, 0, 0), (0.04, 0, 0), (0, 0.04, 0), (0, 0, 0.04)),
      ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)),
    )
    limits = ParallelJawLimits(0, 0.1)
    self.assertEqual(
      propose_contact_pairs(mesh, limits, options=ContactPairOptions(0, 20, 0)),
      [],
    )
    options = ContactPairOptions(
      0, 20, 0, max_normal_angle_rad=math.radians(60)
    )
    pairs = propose_contact_pairs(mesh, limits, options=options)
    self.assertGreater(len(pairs), 0)
    self.assert_pairs_valid(mesh, pairs, limits, options)


if __name__ == "__main__":
  unittest.main()
