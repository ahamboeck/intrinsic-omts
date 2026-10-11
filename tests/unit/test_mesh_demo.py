"""The mesh demo is deliberately local-only, never robot-feasible selection."""

import math
import unittest

from src.grasping.mesh_inputs import cuboid_mesh
from tools.grasping.mesh_demo import plan_demo


class MeshDemoTest(unittest.TestCase):
  def test_lab_stock_small_estimate_tilt_and_no_execution_claim(self):
    mesh = cuboid_mesh((0.0508, 0.127, 0.0762))
    angle = math.radians(1)
    c, s = math.cos(angle), math.sin(angle)
    pose = {
      "rotation_basis_columns": ((c, 0, -s), (0, 1, 0), (s, 0, c)),
      "translation_m": (0.27, 0.27, 0.69),
    }
    cad = {"vertices": mesh.vertices, "triangles": mesh.triangles}
    report = plan_demo(cad, pose)
    self.assertGreater(report["locally_accepted"], 0)
    self.assertFalse(report["execution_ready"])
    self.assertEqual(report["robot_scene_feasibility"], "NOT_CHECKED")
    self.assertEqual(plan_demo(cad, pose, 0)["proposals"], 0)


if __name__ == "__main__":
  unittest.main()
