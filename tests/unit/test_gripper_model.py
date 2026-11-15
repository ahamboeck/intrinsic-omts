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

"""The cells' `gripper.model` matches the gripper's simulated geometry."""

import glob
import os
import xml.etree.ElementTree as ET

from absl.testing import absltest

from src.core.config import load_app_config

# Patched by `bazel/patches/robotiq_hande_finger_offset.patch`.
_HANDE_SDF = (
  "intrinsic_control/intrinsic/models/assets/standard_tooling/"
  "robotiq_hande/robotiq_hande/robotiq_hande.sdf"
)
_HANDE_CELLS = (
  "configs/lab_bb_01/app_config.yaml",
  "configs/omts/app_config.yaml",
)


def _find_runfile(suffix: str) -> str:
  roots = [os.environ.get("TEST_SRCDIR", ""), ".."]
  for root in filter(None, roots):
    found = glob.glob(os.path.join(root, "*", suffix))
    if found:
      return found[0]
  raise FileNotFoundError(suffix)


def _pose(element: ET.Element) -> list[float]:
  pose = element.find("pose")
  values = [float(v) for v in pose.text.split()] if pose is not None else []
  return values or [0.0] * 6


def _hande_jaw_geometry() -> tuple[float, float, float]:
  """Returns (min width, max width, contact depth) from the SDF colliders.

  Each finger touches the part with a fingertip sphere. Its inner surface is
  `|x| - radius` from the gripper's center line when open, and closes by the
  finger joint's upper limit.
  """
  model = ET.parse(_find_runfile(_HANDE_SDF)).getroot().find("model")
  body = model.find("link[@name='gripper_body']")
  if any(_pose(body)):
    raise AssertionError("gripper_body is expected at the model origin.")
  open_gap = closed_gap = 0.0
  tip_heights = set()
  for finger in ("gripper_finger1", "gripper_finger2"):
    link = model.find(f"link[@name='{finger}']")
    (tip,) = [
      collision
      for collision in link.findall("collision")
      if collision.find("geometry/sphere") is not None
    ]
    radius = float(tip.find("geometry/sphere/radius").text)
    x = _pose(link)[0] + _pose(tip)[0]
    (joint,) = [
      joint
      for joint in model.findall("joint")
      if joint.findtext("child") == finger
    ]
    axis_x = float(joint.find("axis/xyz").text.split()[0])
    if axis_x * x >= 0:
      raise AssertionError(f"{finger} does not close toward the center.")
    travel = float(joint.find("axis/limit/upper").text) - float(
      joint.find("axis/limit/lower").text
    )
    open_gap += abs(x) - radius
    closed_gap += abs(x) - radius - travel
    tip_heights.add(round(_pose(link)[2] + _pose(tip)[2], 9))
  (tip_height,) = tip_heights
  tool_frame = model.find("frame[@name='tool_frame']")
  return closed_gap, open_gap, tip_height - _pose(tool_frame)[2]


class GripperModelTest(absltest.TestCase):
  def test_hande_cells_match_the_hande_sdf(self):
    min_width, max_width, contact_depth = _hande_jaw_geometry()
    for path in _HANDE_CELLS:
      with self.subTest(cell=path):
        jaw = load_app_config(path).gripper.model.parallel_jaw
        self.assertAlmostEqual(jaw.min_width_m, min_width, places=6)
        self.assertAlmostEqual(jaw.max_width_m, max_width, places=6)
        self.assertAlmostEqual(jaw.contact_depth_m, contact_depth, places=6)


if __name__ == "__main__":
  absltest.main()
