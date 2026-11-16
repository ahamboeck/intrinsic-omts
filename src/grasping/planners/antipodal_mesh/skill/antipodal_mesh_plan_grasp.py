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

"""Skill that plans an antipodal mesh grasp and checks it with the motion planner."""

import numpy as np
from absl import logging
from intrinsic.motion_planning import motion_planner_client
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface
from intrinsic.world.proto import collision_settings_pb2

from src.grasping import feasibility, frames, workpiece_mesh
from src.grasping.planners.antipodal_mesh import candidates
from src.grasping.planners.antipodal_mesh.skill import (
  antipodal_mesh_plan_grasp_pb2,
)

# Status code for "no candidate passed the motion planner checks".
NO_FEASIBLE_GRASP = 10400


def _value(params, field, default):
  return getattr(params, field) if params.HasField(field) else default


class AntipodalMeshPlanGrasp(skill_interface.Skill):
  """Plans a top-down mesh grasp and writes it to the grasp frames."""

  def get_footprint(self, request, context):
    del request, context
    return footprint_pb2.Footprint(lock_the_universe=True)

  def execute(self, request, context):
    params = request.params
    if not params.HasField("gripper"):
      raise skill_interface.InvalidSkillParametersError(
        "The parallel-jaw gripper geometry is required."
      )
    world = context.object_world
    root = world.root
    workpiece = world.get_object(params.workpiece_object)
    world_t_object = world.get_transform(root, workpiece).matrix4x4()
    world_t_tool = world.get_transform(
      root, world.get_frame(params.tool_frame, params.tool_object)
    ).matrix4x4()

    vertices, triangles = workpiece_mesh.workpiece_mesh(
      workpiece, context.geometry_service
    )
    logging.info(
      "Workpiece mesh: %d triangles, extent %s m.",
      len(triangles),
      np.ptp(vertices, axis=0).round(4),
    )
    found = candidates.top_down_grasps(
      vertices,
      triangles,
      world_t_object,
      params.gripper,
      pregrasp_distance_m=_value(params, "pregrasp_distance_m", 0.1),
      max_tilt_deg=_value(params, "max_approach_tilt_deg", 10.0),
      preferred_jaw_axis=world_t_tool[:3, 0],
    )
    settings = collision_settings_pb2.CollisionSettings(
      minimum_margin=_value(params, "collision_margin_m", 0.002)
    )
    # As in Core's motion planning skills: prefer the resolved service.
    planner = None
    if params.HasField("motion_planner_service"):
      planner = motion_planner_client.get_motion_planner_service_asset_client(
        world.world_id, params.motion_planner_service
      )
    planner = planner or context.motion_planner
    checked = found[: _value(params, "max_checked_candidates", 10)]
    for index, candidate in enumerate(checked, 1):
      if not feasibility.grasp_is_feasible(
        planner,
        params.robot_object,
        params.tool_object,
        params.tool_frame,
        candidate.grasp,
        candidate.pregrasp,
        settings,
      ):
        continue
      frames.set_root_frames(
        world,
        {
          params.grasp_frame: candidate.grasp,
          params.pregrasp_frame: candidate.pregrasp,
        },
      )
      logging.info(
        "Selected mesh grasp %d of %d (width %.4f m).",
        index,
        len(found),
        candidate.width_m,
      )
      return antipodal_mesh_plan_grasp_pb2.AntipodalMeshPlanGraspResult(
        width_m=candidate.width_m,
        num_candidates=len(found),
        num_checked=index,
      )
    raise skill_interface.SkillError(
      NO_FEASIBLE_GRASP,
      f"None of the {len(checked)} checked mesh grasps ({len(found)} found) "
      "is reachable without collision.",
    )
