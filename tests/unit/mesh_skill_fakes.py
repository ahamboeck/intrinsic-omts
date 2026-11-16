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

"""Fake Object World and motion planner for testing mesh grasp skills."""

import types

import grpc
from google.protobuf import json_format
from intrinsic.math.python import data_types
from intrinsic.world.proto import object_world_service_pb2

# A 40 x 100 x 30 mm box: graspable across x only.
BOX_MESH = {
  "vertices": [
    c * s
    for corner in (
      (-1, -1, -1),
      (1, -1, -1),
      (1, 1, -1),
      (-1, 1, -1),
      (-1, -1, 1),
      (1, -1, 1),
      (1, 1, 1),
      (-1, 1, 1),
    )
    for c, s in zip(corner, (0.02, 0.05, 0.015), strict=True)
  ],
  "faces": [
    *(0, 2, 1, 0, 3, 2),
    *(4, 5, 6, 4, 6, 7),
    *(0, 1, 5, 0, 5, 4),
    *(1, 2, 6, 1, 6, 5),
    *(2, 3, 7, 2, 7, 6),
    *(3, 0, 4, 3, 4, 7),
  ],
}


def box_object(geometry, link_offset=(0.0, 0.0, 0.0), shape_transform=None):
  """An Object World object proto whose link entity carries the box mesh."""
  shape = {"geometry": geometry}
  if shape_transform:
    shape["refTShape"] = shape_transform
  x, y, z = link_offset
  return json_format.ParseDict(
    {
      "name": "box",
      "rootEntityId": "root_entity",
      "entities": {
        "root_entity": {"id": "root_entity"},
        "link": {
          "id": "link",
          "parentId": "root_entity",
          "parentTThis": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"w": 1.0},
          },
          "geometryComponent": {
            "namedGeometries": {
              "Intrinsic_Visual": {"namedGeometries": {"box": shape}}
            }
          },
        },
      },
    },
    object_world_service_pb2.Object(),
  )


INLINE_BOX = {
  "inlineGeometryData": {"exactGeometry": {"triangleMesh": BOX_MESH}}
}


class World:
  """Object World stand-in: the box at (0.3, 0.2, 0.7), tool at the origin."""

  def __init__(self):
    self.root = types.SimpleNamespace(frame_names=["grasp", "pre_grasp"])
    self.box = types.SimpleNamespace(proto=box_object(INLINE_BOX))
    self.object_pose = data_types.Pose3(translation=[0.3, 0.2, 0.7])
    self.updates = {}
    self.created = []

  def get_object(self, name):
    assert name == "box"
    return self.box

  def get_frame(self, frame, owner):
    if owner == "root" and frame not in self.root.frame_names:
      raise ValueError(f"Missing frame {frame}")
    return ("frame", owner, frame)

  def get_transform(self, node_a, node_b):
    del node_a
    if node_b is self.box:
      return self.object_pose
    return data_types.Pose3()

  def update_transform(self, node_a, node_b, a_t_b):
    del node_a
    self.updates[node_b[2]] = a_t_b.matrix4x4()

  def create_frame(self, frame_name, parent, parent_t_frame):
    assert parent is self.root or parent[2] in self.root.frame_names
    assert frame_name not in self.root.frame_names
    self.root.frame_names.append(frame_name)
    self.created.append(frame_name)
    self.updates[frame_name] = parent_t_frame.matrix4x4()


class PlanningError(grpc.RpcError):
  def code(self):
    return grpc.StatusCode.FAILED_PRECONDITION

  def details(self):
    return "No collision-free Cartesian approach"


class Planner:
  """Motion planner stand-in that fails IK for the first `ik_failures` calls."""

  def __init__(
    self, ik_failures=0, collides=False, plan_failures=0, empty_path=False
  ):
    self.ik_failures = ik_failures
    self.collides = collides
    self.ik_calls = 0
    self.plan_failures = plan_failures
    self.empty_path = empty_path
    self.paths = []

  def compute_ik(self, robot, target, starting_joints=None, options=None):
    del robot, target, starting_joints
    assert options.collision_settings.minimum_margin > 0
    self.ik_calls += 1
    solved = self.ik_calls > self.ik_failures
    return types.SimpleNamespace(
      solutions=[[float(self.ik_calls)] * 6] if solved else []
    )

  def check_collisions(self, robot, waypoints, options):
    del robot, options
    assert len(waypoints) == 2
    return types.SimpleNamespace(has_collision=self.collides)

  def plan_path(self, robot, specification):
    self.paths.append((robot, specification))
    if self.collides or len(self.paths) <= self.plan_failures:
      raise PlanningError()
    return types.SimpleNamespace(
      path=types.SimpleNamespace(
        waypoints=[] if self.empty_path else [robot.start_configuration]
      )
    )
