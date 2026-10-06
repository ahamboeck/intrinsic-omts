"""Pure top-down frames: no SDK, IK, scene or physical-grasp claims."""

import math
import unittest
from dataclasses import FrozenInstanceError

from src.grasping.geometry import RigidTransform
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.poses import propose_top_down_grasps
from src.grasping.types import ContactPair, SurfaceContact


def pair(first=(0, 0, 0), second=(0.02, 0, 0)):
  direction = tuple(
    (b - a) / math.dist(first, second)
    for a, b in zip(first, second, strict=True)
  )
  return ContactPair(
    SurfaceContact(first, tuple(-v for v in direction), 0),
    SurfaceContact(second, direction, 1),
  )


class GraspPosesTest(unittest.TestCase):
  def proposals(self, contacts=None, **kwargs):
    return propose_top_down_grasps(
      (contacts or pair(),),
      hande_collision_profile(),
      world_T_object=kwargs.pop("world_T_object", RigidTransform()),
      pregrasp_distance_m=kwargs.pop("pregrasp_distance_m", 0.1),
      open_width_m=kwargs.pop("open_width_m", 0.06),
      **kwargs,
    )

  def assert_vec(self, actual, expected):
    for a, b in zip(actual, expected, strict=True):
      self.assertAlmostEqual(a, b, places=12)

  def test_top_down_center_is_right_handed_with_two_yaw_choices(self):
    first, second = self.proposals()
    self.assert_vec(first.object_T_center.translation, (0.01, 0, 0))
    self.assertEqual(
      first.object_T_center.rotation, ((1, 0, 0), (0, -1, 0), (0, 0, -1))
    )
    self.assertEqual(
      second.object_T_center.rotation, ((-1, 0, 0), (0, 1, 0), (0, 0, -1))
    )
    self.assertEqual(second.contacts.first, first.contacts.second)

  def test_model_tcp_offset_and_pregrasp_are_not_contact_center(self):
    proposal = self.proposals()[0]
    self.assert_vec(proposal.object_T_tcp.translation, (0.01, 0, 0.022))
    self.assert_vec(
      proposal.object_T_pregrasp_tcp.translation, (0.01, 0, 0.122)
    )
    self.assertEqual(
      proposal.object_T_tcp.rotation, proposal.object_T_pregrasp_tcp.rotation
    )

  def test_arbitrary_nonidentity_tcp_rotation_and_offset_are_composed(self):
    from dataclasses import replace

    profile = replace(
      hande_collision_profile(),
      center_T_tcp=RigidTransform(
        ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (0.004, 0.005, -0.022)
      ),
    )
    result = propose_top_down_grasps(
      (pair(),),
      profile,
      world_T_object=RigidTransform(),
      pregrasp_distance_m=0.1,
      open_width_m=0.06,
    )[0]
    self.assert_vec(result.object_T_tcp.translation, (0.014, -0.005, 0.022))
    self.assertEqual(
      result.object_T_tcp.rotation, ((0, -1, 0), (-1, 0, 0), (0, 0, -1))
    )

  def test_vertical_and_tilted_closing_lines_are_rejected_not_tilted(self):
    self.assertEqual(self.proposals(pair(second=(0, 0, 0.02))), [])
    self.assertEqual(self.proposals(pair(second=(0.02, 0, 0.001))), [])

  def test_top_down_means_world_down_for_a_tilted_object(self):
    world_T_object = RigidTransform(
      ((0, 0, -1), (0, 1, 0), (1, 0, 0)), (0.3, -0.2, 0.1)
    )
    # Object Z is horizontal in the world, while object X is vertical.
    result = self.proposals(
      pair(second=(0, 0, 0.02)), world_T_object=world_T_object
    )[0]
    self.assert_vec(
      world_T_object.rotate(result.object_T_center.rotation[2]), (0, 0, -1)
    )
    grasp = world_T_object.compose(result.object_T_tcp)
    pregrasp = world_T_object.compose(result.object_T_pregrasp_tcp)
    self.assert_vec(
      tuple(
        b - a
        for a, b in zip(grasp.translation, pregrasp.translation, strict=True)
      ),
      (0, 0, 0.1),
    )
    self.assertEqual(self.proposals(world_T_object=world_T_object), [])

  def test_object_coordinate_change_preserves_world_poses(self):
    change = RigidTransform(
      ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (0.05, -0.02, 0.03)
    )
    original = pair()
    transformed = ContactPair(
      *(
        SurfaceContact(
          change.apply(c.point),
          change.rotate(c.outward_normal),
          c.triangle_index,
        )
        for c in (original.first, original.second)
      )
    )
    before = self.proposals(original)[0]
    after = self.proposals(transformed, world_T_object=change.inverse())[0]
    for field in ("object_T_center", "object_T_tcp", "object_T_pregrasp_tcp"):
      actual = change.inverse().compose(getattr(after, field))
      expected = getattr(before, field)
      self.assert_vec(actual.translation, expected.translation)
      for a, b in zip(actual.rotation, expected.rotation, strict=True):
        self.assert_vec(a, b)

  def test_opening_is_explicit_and_must_allow_approach_clearance(self):
    self.assertEqual(self.proposals(open_width_m=0.02), [])
    self.assertEqual(self.proposals(open_width_m=0.015), [])
    self.assertEqual(self.proposals(pair(second=(0.005, 0, 0))), [])
    self.assertEqual(self.proposals(open_width_m=0.04)[0].open_width_m, 0.04)

  def test_invalid_inputs_fail_and_outputs_are_immutable(self):
    for value in (0, -1, math.nan, math.inf, True):
      with self.subTest(distance=value), self.assertRaises(ValueError):
        self.proposals(pregrasp_distance_m=value)
    for value in (0, 0.07, math.nan, True):
      with self.subTest(opening=value), self.assertRaises(ValueError):
        self.proposals(open_width_m=value)
    with self.assertRaises(FrozenInstanceError):
      self.proposals()[0].open_width_m = 0.04

  def test_rigid_transform_rejects_invalid_rotations_and_mutable_storage(self):
    for rotation in (
      ((1, 0, 0), (0, 1, 0), (0, 0, -1)),
      ((2, 0, 0), (0, 1, 0), (0, 0, 1)),
      ((1, 0, 0), (1, 0, 0), (0, 0, 1)),
      [(1, 0, 0), (0, 1, 0), (0, 0, 1)],
    ):
      with self.subTest(rotation=rotation), self.assertRaises(ValueError):
        RigidTransform(rotation)
    with self.assertRaises(ValueError):
      RigidTransform(translation=(0, math.inf, 0))
    transform = RigidTransform(
      ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (0.1, -0.2, 0.3)
    )
    self.assert_vec(
      transform.inverse().apply(transform.apply((1, 2, 3))), (1, 2, 3)
    )


if __name__ == "__main__":
  unittest.main()
