"""World-space overlay geometry independent of ROS/SDK and real execution."""

import unittest

from tools.grasping.pose_overlay import marker_specs, split_marker_specs


class PoseOverlayTest(unittest.TestCase):
  def test_column_rotation_and_translation_applied_exactly_once(self):
    cad = dict(
      vertices=[[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.02, 0.0]],
      triangles=[[0, 1, 2]],
    )
    value = dict(
      translation_m=[1.0, 2.0, 3.0],
      rotation_basis_columns=[
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
      ],
    )
    report = dict(
      actual_after_evaluation_only=value,
      fresh_world_object=value,
      finished_at_utc="2026-10-11T22:40:00Z",
    )
    specs = marker_specs(cad, report)
    self.assertEqual(len(specs), 10)
    self.assertEqual(
      specs[0]["points"], [(1.0, 2.0, 3.0), (1.0, 2.05, 3.0), (0.98, 2.0, 3.0)]
    )
    self.assertEqual(specs[1]["points"], [(1.0, 2.0, 3.0), (1.0, 2.09, 3.0)])
    self.assertEqual(specs[3]["points"], [(1.0, 2.0, 3.0), (1.0, 2.0, 3.09)])
    self.assertEqual(specs[0]["points"], specs[5]["points"])
    self.assertNotEqual(specs[0]["namespace"], specs[5]["namespace"])
    self.assertEqual({s["frame_id"] for s in specs}, {"root"})
    self.assertIn("SNAPSHOT", specs[4]["text"])
    self.assertIn(report["finished_at_utc"], specs[4]["text"])
    groups = split_marker_specs(specs)
    for name, color in (
      ("actual", (0.1, 1.0, 0.2)),
      ("perceived", (1.0, 0.1, 0.8)),
    ):
      self.assertEqual(len(groups[name]), 5)
      self.assertEqual(
        {s["namespace"] for s in groups[name]}, {"omts_pose_snapshot/" + name}
      )
      self.assertEqual(groups[name][0]["color"][:3], color)
      self.assertEqual([s["id"] for s in groups[name]], list(range(5)))

  def test_invalid_rotation_is_rejected_not_silently_normalized(self):
    cad = dict(
      vertices=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
      triangles=[[0, 1, 2]],
    )
    value = dict(
      translation_m=[0.0, 0.0, 0.0],
      rotation_basis_columns=[[1.0, 0.0, 0.0]] * 3,
    )
    with self.assertRaisesRegex(ValueError, "orthonormal"):
      marker_specs(
        cad,
        dict(actual_after_evaluation_only=value, finished_at_utc="snapshot"),
      )


if __name__ == "__main__":
  unittest.main()
