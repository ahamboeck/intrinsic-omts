"""Conservative gripper/object checks, including obstacles between endpoints."""

import math
import unittest
from dataclasses import replace

from src.grasping.contact_pairs import propose_contact_pairs
from src.grasping.geometry import RigidTransform
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.local_checks import check_local_grasp
from src.grasping.poses import propose_top_down_grasps
from src.grasping.profile import CollisionBox, ParallelJawProfile
from src.grasping.types import (
  ContactPair,
  ContactPairOptions,
  ParallelJawLimits,
  SurfaceContact,
  TriangleMesh,
)


def extruded(polygon, faces, height):
  n = len(polygon)
  vertices = tuple((x, y, z) for z in (0, height) for x, y in polygon)
  triangles = [(c, b, a) for a, b, c in faces]
  triangles += [(a + n, b + n, c + n) for a, b, c in faces]
  for a in range(n):
    b = (a + 1) % n
    triangles.extend(((a, b, b + n), (a, b + n, a + n)))
  return TriangleMesh(vertices, tuple(triangles))


def box(width=0.02, depth=0.01, height=0.01):
  return extruded(
    ((0, 0), (width, 0), (width, depth), (0, depth)),
    ((0, 1, 2), (0, 2, 3)),
    height,
  )


def contacts(mesh, first, second):
  """Fixture contact provenance on X planes, using independent barycentrics."""
  found = []
  for point, direction in ((first, -1), (second, 1)):
    for index, triangle in enumerate(mesh.triangles):
      a, b, c = [mesh.vertices[i] for i in triangle]
      if not all(abs(v[0] - point[0]) < 1e-12 for v in (a, b, c)):
        continue
      area = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
      if area * direction <= 0:
        continue
      u = (
        (point[1] - a[1]) * (c[2] - a[2]) - (point[2] - a[2]) * (c[1] - a[1])
      ) / area
      v = (
        (b[1] - a[1]) * (point[2] - a[2]) - (b[2] - a[2]) * (point[1] - a[1])
      ) / area
      if u >= -1e-10 and v >= -1e-10 and u + v <= 1 + 1e-10:
        found.append(SurfaceContact(point, (direction, 0, 0), index))
        break
    else:
      raise AssertionError("fixture contact not on outward X face")
  return ContactPair(*found)


def envelope(name, xyz, half=(0.003, 0.003, 0.003), pad=False):
  return CollisionBox(name, RigidTransform(translation=xyz), half, pad)


def synthetic_profile(body=(0, 0, -0.02), max_open=0.06):
  return ParallelJawProfile(
    "synthetic-only",
    ParallelJawLimits(0.001, max_open),
    RigidTransform(),
    (envelope("body", body, (0.005, 0.005, 0.002)),),
    (envelope("positive_pad", (max_open / 2 + 0.003, 0, 0), pad=True),),
    (envelope("negative_pad", (-max_open / 2 - 0.003, 0, 0), pad=True),),
  )


def proposal(
  mesh, profile, first=(0, 0.005, 0.005), second=(0.02, 0.005, 0.005)
):
  return propose_top_down_grasps(
    (contacts(mesh, first, second),),
    profile,
    world_T_object=RigidTransform(),
    pregrasp_distance_m=0.1,
    open_width_m=profile.limits.max_opening_m,
  )[0]


class LocalGraspChecksTest(unittest.TestCase):
  def test_concave_search_to_top_down_local_checks_is_deterministic(self):
    mesh = extruded(
      ((0, 0), (0.06, 0), (0.06, 0.02), (0.02, 0.02), (0.02, 0.06), (0, 0.06)),
      ((0, 1, 3), (1, 2, 3), (0, 3, 5), (3, 4, 5)),
      0.015,
    )
    profile = hande_collision_profile()
    options = ContactPairOptions(
      seed=42, max_pairs=20, opening_clearance_m=0.001
    )

    def run():
      pairs = propose_contact_pairs(mesh, profile.limits, options=options)
      poses = propose_top_down_grasps(
        tuple(pairs),
        profile,
        world_T_object=RigidTransform(),
        pregrasp_distance_m=0.1,
        open_width_m=0.06,
      )
      return (
        pairs,
        poses,
        [check_local_grasp(mesh, pose, profile) for pose in poses],
      )

    first = run()
    self.assertEqual(first, run())
    pairs, poses, results = first
    self.assertTrue(pairs)
    self.assertTrue(poses)
    self.assertLessEqual(len(poses), 2 * len(pairs))
    self.assertTrue(any(result.accepted for result in results))
    self.assertTrue(any(not result.accepted for result in results))
    for pose in poses:
      self.assertEqual(pose.object_T_center.rotation[2], (0, 0, -1))

  def test_slanted_contact_face_is_not_a_valid_planar_pad_contact(self):
    mesh = extruded(
      ((0, 0), (0.02, 0), (0.021, 0.01), (0, 0.01)),
      ((0, 1, 2), (0, 2, 3)),
      0.01,
    )
    normal = (1 / math.sqrt(1.01), -0.1 / math.sqrt(1.01), 0)
    pair = ContactPair(
      SurfaceContact((0, 0.005, 0.005), (-1, 0, 0), 10),
      SurfaceContact((0.0205, 0.005, 0.005), normal, 6),
    )
    profile = synthetic_profile()
    grasp = propose_top_down_grasps(
      (pair,),
      profile,
      world_T_object=RigidTransform(),
      pregrasp_distance_m=0.1,
      open_width_m=0.06,
    )[0]
    result = check_local_grasp(mesh, grasp, profile)
    self.assertFalse(result.accepted)
    self.assertEqual(result.stage, "contacts")

  def test_tcp_calibration_does_not_move_collision_geometry(self):
    mesh, profile = box(), hande_collision_profile()
    profile = replace(
      profile,
      center_T_tcp=RigidTransform(
        ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
        (0.003, 0.004, -0.034),
      ),
    )
    self.assertTrue(
      check_local_grasp(mesh, proposal(mesh, profile), profile).accepted
    )

  def test_hande_encloses_all_source_primitives_and_includes_link_patch(self):
    profile = hande_collision_profile()
    self.assertEqual(
      (
        len(profile.body_boxes),
        len(profile.positive_jaw_boxes),
        len(profile.negative_jaw_boxes),
      ),
      (5, 3, 3),
    )
    self.assertEqual(profile.limits, ParallelJawLimits(0.01, 0.06))
    self.assertEqual(profile.center_T_tcp.translation, (0, 0, -0.022))
    self.assertEqual(
      profile.positive_jaw_boxes[0].center_T_box.translation, (0.033, 0, 0)
    )
    self.assertEqual(
      profile.negative_jaw_boxes[0].center_T_box.translation, (-0.033, 0, 0)
    )
    self.assertEqual(
      profile.body_boxes[0].half_extents_m, (0.037544, 0.037544, 0.09574 / 2)
    )
    self.assertEqual(
      profile.positive_jaw_boxes[2].half_extents_m,
      (0.005967 / 2, 0.043104 / 2, 0.0164 / 2),
    )

  def test_simple_box_accepts_pad_tangency_without_disabling_finger_collisions(
    self,
  ):
    mesh, profile = box(), hande_collision_profile()
    result = check_local_grasp(mesh, proposal(mesh, profile), profile)
    self.assertTrue(result.accepted, result)
    self.assertEqual(result.reason, "passed declared local checks")

  def test_final_body_collision_rejects(self):
    mesh, profile = box(), synthetic_profile(body=(0, 0, 0))
    result = check_local_grasp(mesh, proposal(mesh, profile), profile)
    self.assertFalse(result.accepted)
    self.assertEqual((result.stage, result.collider), ("approach", "body"))

  def test_body_tangency_is_not_treated_as_allowed_pad_contact(self):
    mesh, profile = box(), synthetic_profile(body=(0, 0, -0.007))
    # The lower body face is exactly at the object's top Z=10 mm.
    result = check_local_grasp(mesh, proposal(mesh, profile), profile)
    self.assertFalse(result.accepted)
    self.assertEqual(result.collider, "body")

  def test_rotated_collider_sweep_is_not_reduced_to_endpoint_checks(self):
    mesh, profile = box(), synthetic_profile()
    angle = math.pi / 4
    c, s = math.cos(angle), math.sin(angle)
    rotated = CollisionBox(
      "rotated_body",
      RigidTransform(((c, 0, -s), (0, 1, 0), (s, 0, c)), (0, 0, 0.05)),
      (0.005, 0.005, 0.002),
    )
    profile = replace(profile, body_boxes=(rotated,))
    result = check_local_grasp(mesh, proposal(mesh, profile), profile)
    self.assertFalse(result.accepted)
    self.assertEqual(
      (result.stage, result.collider), ("approach", "rotated_body")
    )

  def test_contained_collider_is_rejected_even_without_surface_intersection(
    self,
  ):
    mesh, profile = box(), synthetic_profile(body=(0, 0, 0))
    profile = replace(
      profile,
      body_boxes=(envelope("tiny_inside", (0, 0, 0), (0.001, 0.001, 0.001)),),
    )
    # A tiny approach sweep also fits fully inside the object.
    grasp = proposal(mesh, profile)
    grasp = replace(
      grasp,
      object_T_pregrasp_tcp=RigidTransform(
        grasp.object_T_tcp.rotation, (0.01, 0.005, 0.0051)
      ),
    )
    result = check_local_grasp(mesh, grasp, profile)
    self.assertFalse(result.accepted)
    self.assertEqual(result.collider, "tiny_inside")

  def test_approach_sweep_rejects_collision_between_clear_endpoints(self):
    mesh, profile = box(), synthetic_profile(body=(0, 0, 0.05))
    grasp = proposal(mesh, profile)
    # Body final Z=-45 mm and pregrasp Z=55 mm, both outside [0,10] mm.
    self.assertLess(grasp.object_T_center.apply((0, 0, 0.05))[2] + 0.002, 0)
    self.assertGreater(
      grasp.object_T_center.apply((0, 0, -0.05))[2] - 0.002, 0.01
    )
    result = check_local_grasp(mesh, grasp, profile)
    self.assertFalse(result.accepted)
    self.assertEqual((result.stage, result.collider), ("approach", "body"))

  def test_closure_sweep_rejects_a_second_arm_between_clear_open_and_closed_jaws(
    self,
  ):
    mesh = extruded(
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
      0.01,
    )
    profile = synthetic_profile(max_open=0.12)
    grasp = proposal(
      mesh, profile, first=(0, 0.04, 0.005), second=(0.02, 0.04, 0.005)
    )
    # Positive pad is beyond X=70 mm when open, at X=20..26 mm when closed;
    # the right arm at X=40..60 mm is crossed only during closure.
    result = check_local_grasp(mesh, grasp, profile)
    self.assertFalse(result.accepted)
    self.assertEqual(
      (result.stage, result.collider), ("closure", "positive_pad")
    )

  def test_noncontact_jaw_geometry_cannot_be_exempted(self):
    mesh, profile = box(), synthetic_profile()
    profile = replace(
      profile,
      positive_jaw_boxes=profile.positive_jaw_boxes
      + (envelope("penetrating_finger", (0.018, 0, 0), (0.002, 0.002, 0.002)),),
    )
    result = check_local_grasp(mesh, proposal(mesh, profile), profile)
    self.assertFalse(result.accepted)
    self.assertEqual(result.collider, "penetrating_finger")

  def test_contact_provenance_and_declared_normal_are_verified(self):
    mesh, profile = box(), synthetic_profile()
    grasp = proposal(mesh, profile)
    for contact in (
      replace(grasp.contacts.first, triangle_index=100),
      replace(grasp.contacts.first, point=(0, 1, 1)),
      replace(grasp.contacts.first, outward_normal=(1, 0, 0)),
    ):
      with self.subTest(contact=contact), self.assertRaises(ValueError):
        check_local_grasp(
          mesh,
          replace(grasp, contacts=ContactPair(contact, grasp.contacts.second)),
          profile,
        )

  def test_inconsistent_tcp_pregrasp_center_or_aperture_are_rejected(self):
    mesh, profile = box(), hande_collision_profile()
    grasp = proposal(mesh, profile)
    invalid = (
      replace(grasp, object_T_tcp=RigidTransform()),
      replace(grasp, object_T_center=RigidTransform()),
      replace(
        grasp,
        object_T_pregrasp_tcp=RigidTransform(
          grasp.object_T_tcp.rotation, (0.11, 0.005, 0.027)
        ),
      ),
      replace(grasp, object_T_pregrasp_tcp=grasp.object_T_tcp),
      replace(grasp, open_width_m=0.07),
    )
    for candidate in invalid:
      with self.subTest(candidate=candidate), self.assertRaises(ValueError):
        check_local_grasp(mesh, candidate, profile)

  def test_solid_containment_and_collision_are_object_coordinate_invariant(
    self,
  ):
    mesh, profile = box(), hande_collision_profile()
    original = proposal(mesh, profile)
    change = RigidTransform(((0, 1, 0), (0, 0, 1), (1, 0, 0)), (0.3, -0.2, 0.1))
    transformed_mesh = TriangleMesh(
      tuple(change.apply(v) for v in mesh.vertices), mesh.triangles
    )
    transformed_contacts = ContactPair(
      *(
        SurfaceContact(
          change.apply(c.point),
          change.rotate(c.outward_normal),
          c.triangle_index,
        )
        for c in (original.contacts.first, original.contacts.second)
      )
    )
    transformed = replace(
      original,
      contacts=transformed_contacts,
      object_T_center=change.compose(original.object_T_center),
      object_T_tcp=change.compose(original.object_T_tcp),
      object_T_pregrasp_tcp=change.compose(original.object_T_pregrasp_tcp),
    )
    self.assertEqual(
      check_local_grasp(mesh, original, profile).accepted,
      check_local_grasp(transformed_mesh, transformed, profile).accepted,
    )

  def test_open_or_inward_mesh_fails_before_collision_checks(self):
    mesh, profile = box(), synthetic_profile()
    grasp = proposal(mesh, profile)
    for bad in (
      TriangleMesh(mesh.vertices, mesh.triangles[:-1]),
      TriangleMesh(
        mesh.vertices, tuple(tuple(reversed(t)) for t in mesh.triangles)
      ),
    ):
      with self.subTest(mesh=bad), self.assertRaises(ValueError):
        check_local_grasp(bad, grasp, profile)

  def test_profile_contracts_reject_mutable_or_unsafe_contact_exemptions(self):
    profile = hande_collision_profile()
    for kwargs in (
      {"body_boxes": list(profile.body_boxes)},
      {"body_boxes": (replace(profile.body_boxes[0], is_contact_pad=True),)},
      {
        "positive_jaw_boxes": (
          replace(profile.positive_jaw_boxes[0], is_contact_pad=False),
        )
      },
      {
        "positive_jaw_boxes": (
          replace(
            profile.positive_jaw_boxes[0],
            center_T_box=RigidTransform(translation=(0, 0, 0)),
          ),
        )
      },
    ):
      with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
        replace(profile, **kwargs)
    for half in ((0, 1, 1), (math.nan, 1, 1), (True, 1, 1)):
      with self.subTest(half=half), self.assertRaises(ValueError):
        envelope("invalid", (0, 0, 0), half)


if __name__ == "__main__":
  unittest.main()
