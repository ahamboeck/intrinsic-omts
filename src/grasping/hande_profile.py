# Numeric geometry derived from Intrinsic's Apache-2.0 Robotiq Hand-E SDF.
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
"""Model-derived Hand-E envelopes, NOT hardware-verified calibration.

Source: intrinsic-core release 20260922.0, robotiq_hande/robotiq_hande.sdf,
plus OMTS robotiq_hande_finger_offset.patch (+/-5 mm at the finger links).
All 11 collision primitives are enclosed, including racks and IO connectors.
Spheres use bounding cubes; cylinders use oriented bounding boxes. This
conservatively encloses the SDF COLLIDERS, not necessarily the visual CAD.

The simplified sphere apexes define a MODEL gap of 10..60 mm across the
0..25 mm inward joint travel. Do not substitute this for the manufacturer's
stroke, measured pad geometry, or the gripper command-to-joint mapping.
"""

import math

from src.grasping.geometry import RigidTransform
from src.grasping.profile import CollisionBox, ParallelJawProfile
from src.grasping.types import ParallelJawLimits


def _box(name, xyz, half, rpy=(0, 0, 0), *, pad=False):
  roll, pitch, yaw = rpy
  cr, sr, cp, sp, cy, sy = (
    math.cos(roll),
    math.sin(roll),
    math.cos(pitch),
    math.sin(pitch),
    math.cos(yaw),
    math.sin(yaw),
  )
  rotation = (
    (cy * cp, sy * cp, -sp),
    (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
    (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
  )
  # SDF poses use body coordinates; grasp center is at sphere-apex Z=150 mm.
  return CollisionBox(
    name, RigidTransform(rotation, (xyz[0], xyz[1], xyz[2] - 0.15)), half, pad
  )


def hande_collision_profile() -> ParallelJawProfile:
  """Return a versioned numeric snapshot; no SDF/SDK parsing at runtime."""
  return ParallelJawProfile(
    name="hande-core-20260922.0-omts-finger-offset-v1",
    limits=ParallelJawLimits(0.01, 0.06),
    # Body_T_center=(0,0,.150); body_T_tool_frame=(0,0,.128).
    center_T_tcp=RigidTransform(translation=(0, 0, -0.022)),
    body_boxes=(
      _box(
        "body",
        (0, 0, 0.06177),
        (0.037544, 0.037544, 0.09574 / 2),
        (0, 0, -1.570796),
      ),
      _box("io_coupler", (0, 0, 0.00545), (0.0379, 0.0379, 0.0169 / 2)),
      _box(
        "io_box1",
        (-0.000733, 0.054862, 0.005695),
        (0.011817 / 2, 0.025604 / 2, 0.010486 / 2),
        (0, 1.570796, 0),
      ),
      _box(
        "io_box2",
        (-0.000739, 0.037415, 0.00575),
        (0.005041 / 2, 0.011557 / 2, 0.0095 / 2),
      ),
      _box(
        "io_cylinder",
        (0.009816, 0.061755, 0.005695),
        (0.005, 0.005, 0.008147 / 2),
        (0, 1.570796, 0),
      ),
    ),
    positive_jaw_boxes=(
      _box("finger1_pad", (0.033, 0, 0.15), (0.003, 0.003, 0.003), pad=True),
      _box("finger1_back", (0.038, 0, 0.127), (0.0025, 0.01, 0.01)),
      _box(
        "finger1_rack",
        (0.018949, -0.01425, 0.1033),
        (0.005967 / 2, 0.043104 / 2, 0.0164 / 2),
        (0, 0, -1.570796),
      ),
    ),
    negative_jaw_boxes=(
      _box("finger2_pad", (-0.033, 0, 0.15), (0.003, 0.003, 0.003), pad=True),
      _box("finger2_back", (-0.038, 0, 0.127), (0.0025, 0.01, 0.01)),
      _box(
        "finger2_rack",
        (-0.019107, 0.01425, 0.1033),
        (0.005967 / 2, 0.04339 / 2, 0.0164 / 2),
        (0, 0, -4.712389),
      ),
    ),
  )
