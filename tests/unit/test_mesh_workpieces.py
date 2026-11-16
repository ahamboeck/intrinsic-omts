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

"""The demo workpieces have the grasp candidates they are meant to have.

Every part has parallel-jaw candidates on its mesh in its scene pose, and the
parts meant for the suction gripper have suction candidates.
"""

import json
import math
import pathlib
import struct

import numpy as np
from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.world.proto import object_world_updates_pb2

from src.core.config import load_app_config
from src.grasping import workpiece_mesh
from src.grasping.planners.antipodal_mesh import candidates
from src.grasping.planners.suction_mesh import candidates as suction

# (workpiece, scene updates placing it, its visual mesh)
_WORKPIECES = (
  (
    "raw_stock_2x3x5",
    "configs/lab_bb_01/scene.updates.pbtxt",
    "models/raw_stock_2x3x5/base_visual.glb",
  ),
  (
    "banana",
    "configs/lab_bb_01_objects/banana.updates.pbtxt",
    "models/banana/banana.glb",
  ),
  (
    "bracket",
    "configs/lab_bb_01_objects/bracket.updates.pbtxt",
    "models/bracket/bracket.glb",
  ),
  (
    "cover_plate",
    "configs/lab_bb_01_objects/cover_plate.updates.pbtxt",
    "models/cover_plate/cover_plate.glb",
  ),
)

# Parts the EPick's 40 mm cup can hold. The banana is too round for it, and
# the bracket's flat faces are narrower than the cup.
_SUCTION_WORKPIECES = ("raw_stock_2x3x5", "cover_plate")

_INDEX_TYPES = {5121: "<u1", 5123: "<u2", 5125: "<u4"}


def _read_glb(path):
  """Returns (vertices, triangles) of a .glb holding one triangle mesh."""
  data = pathlib.Path(path).read_bytes()
  json_length = struct.unpack_from("<I", data, 12)[0]
  gltf = json.loads(data[20 : 20 + json_length])
  binary = data[28 + json_length :]

  def read(index, dtype, width):
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    values = np.frombuffer(binary, dtype, accessor["count"] * width, start)
    return values.reshape(-1, width)

  (primitive,) = gltf["meshes"][0]["primitives"]
  vertices = read(primitive["attributes"]["POSITION"], "<f4", 3)
  if "indices" not in primitive:
    return workpiece_mesh.validate_mesh(
      vertices, np.arange(len(vertices)).reshape(-1, 3)
    )
  index_type = _INDEX_TYPES[
    gltf["accessors"][primitive["indices"]]["componentType"]
  ]
  triangles = read(primitive["indices"], index_type, 1).reshape(-1, 3)
  return workpiece_mesh.validate_mesh(vertices, triangles)


def _scene_pose(scene_path, object_name):
  updates = text_format.Parse(
    pathlib.Path(scene_path).read_text(encoding="utf-8"),
    object_world_updates_pb2.ObjectWorldUpdates(),
  )
  for update in updates.updates:
    transform = update.update_transform
    if transform.node_to_update.by_name.object.object_name == object_name:
      q = transform.a_t_b.orientation
      norm = math.hypot(q.x, q.y, q.z, q.w)
      p = transform.a_t_b.position
      return data_types.Pose3(
        data_types.Rotation3(
          data_types.Quaternion(
            [q.x / norm, q.y / norm, q.z / norm, q.w / norm]
          )
        ),
        [p.x, p.y, p.z],
      ).matrix4x4()
  raise AssertionError(f"{object_name} is not placed in {scene_path}")


class MeshWorkpiecesTest(absltest.TestCase):
  def test_workpieces_have_grasp_candidates(self):
    gripper = load_app_config(
      "configs/lab_bb_01/app_config.yaml"
    ).gripper.model.parallel_jaw
    for workpiece, scene_path, mesh_path in _WORKPIECES:
      with self.subTest(workpiece=workpiece):
        found = candidates.top_down_grasps(
          *_read_glb(mesh_path), _scene_pose(scene_path, workpiece), gripper
        )
        self.assertNotEmpty(found)
        for candidate in found:
          self.assertBetween(
            candidate.width_m, gripper.min_width_m, gripper.max_width_m
          )

  def test_suction_candidates_match_the_parts_meant_for_suction(self):
    cup = load_app_config(
      "configs/lab_bb_01/app_config_epick.yaml"
    ).gripper.model.suction
    for workpiece, scene_path, mesh_path in _WORKPIECES:
      with self.subTest(workpiece=workpiece):
        found = suction.top_down_grasps(
          *_read_glb(mesh_path), _scene_pose(scene_path, workpiece), cup
        )
        if workpiece in _SUCTION_WORKPIECES:
          self.assertNotEmpty(found)
        else:
          self.assertEmpty(found)

  def test_cover_plate_cup_avoids_the_cutout_next_to_the_center(self):
    cup = load_app_config(
      "configs/lab_bb_01/app_config_epick.yaml"
    ).gripper.model.suction
    (_, scene_path, mesh_path) = _WORKPIECES[-1]
    world_t_plate = _scene_pose(scene_path, "cover_plate")
    best = suction.top_down_grasps(*_read_glb(mesh_path), world_t_plate, cup)[0]
    # The 40 mm cutout is centered on the plate, so the nearest spot where the
    # whole 40 mm cup is on metal is 40 mm from the center.
    offset = np.abs(best.grasp[:2, 3] - world_t_plate[:2, 3])
    self.assertGreaterEqual(offset.max(), 0.02 + cup.cup_diameter_m / 2 - 1e-9)
    self.assertLess(np.linalg.norm(offset), 0.044)


if __name__ == "__main__":
  absltest.main()
