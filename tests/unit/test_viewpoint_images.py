"""Evaluation-only image projection and panel tests."""

import unittest

import numpy as np

from src.grasping.geometry import RigidTransform
from tools.pose_estimation.viewpoint_images import (
  cad_in_image,
  comparison_image,
)


class ViewpointImagesTest(unittest.TestCase):
  def test_known_projection_and_panels(self):
    cad = dict(
      vertices=[[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0]], triangles=[[0, 1, 2]]
    )
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    k = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]])
    actual = RigidTransform(translation=(0.0, 0.0, 1.0))
    self.assertTrue(cad_in_image(rgb.shape, k, RigidTransform(), actual, cad))
    image = comparison_image(rgb, k, RigidTransform(), actual, [actual], cad)
    self.assertEqual(image.size, (200, 100))
    self.assertEqual(image.getpixel((55, 50)), (0, 255, 70))
    self.assertEqual(image.getpixel((155, 50)), (255, 40, 210))
    for position in ((0.0, 0.0, -1.0), (3.0, 0.0, 1.0)):
      self.assertFalse(
        cad_in_image(
          rgb.shape,
          k,
          RigidTransform(),
          RigidTransform(translation=position),
          cad,
        )
      )


if __name__ == "__main__":
  unittest.main()
