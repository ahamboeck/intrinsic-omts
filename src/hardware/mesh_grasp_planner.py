"""Opt-in Hand-E simulation adapter; embeds pure mesh code in a BT script."""

import inspect
import json
from pathlib import Path

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb

from src.grasping import (
  contact_pairs,
  geometry,
  hande_profile,
  local_checks,
  poses,
  profile,
  runtime_frames,
  types,
)
from src.hardware.grasping import GraspPlannerInterface


def runtime_source():
  """Bundle OMTS modules: the executive's Python environment doesn't install OMTS.

  Use a private namespace rather than shadowing SDK or arbitrary `src` modules.
  Only repository source is executable; CAD stays JSON data in script parameters.
  """
  sources = {
    module.__name__.rsplit(".", 1)[-1]: inspect.getsource(module).replace(
      "src.grasping", "_omts_mesh_pick"
    )
    for module in (
      types,
      geometry,
      profile,
      hande_profile,
      contact_pairs,
      poses,
      local_checks,
      runtime_frames,
    )
  }
  return (
    "import sys, types\n"
    "_pkg = types.ModuleType('_omts_mesh_pick')\n"
    "_pkg.__path__ = []\n"
    "sys.modules['_omts_mesh_pick'] = _pkg\n"
    f"_sources = {sources!r}\n"
    "for _name, _source in _sources.items():\n"
    "    _full = '_omts_mesh_pick.' + _name\n"
    "    _mod = types.ModuleType(_full)\n"
    "    sys.modules[_full] = _mod\n"
    "    exec(compile(_source, _full, 'exec'), _mod.__dict__)\n"
    "sys.modules['_omts_mesh_pick.runtime_frames'].plan_mesh_frames(context, params)\n"
  )


class MeshGraspPlanner(GraspPlannerInterface):
  uses_contact_touchdown = False

  def __init__(self, solution, config, tool_object_name, tool_frame_name):
    if config.mesh_simulation_only is not True:
      raise ValueError("Mesh backend is opt-in for simulation only.")
    self.solution = solution
    self.config = config
    self.tool_object_name = tool_object_name
    self.tool_frame_name = tool_frame_name
    cad = json.loads(Path(config.mesh_cad_json).read_text())
    runtime_frames.mesh_from_json(cad)
    self.cad_json = json.dumps(cad)

  def build_plan_grasp_task(
    self,
    workpiece_object_name,
    parent_object="root",
    grasp_frame_name="grasp",
    pregrasp_frame_name="pre_grasp",
    name=None,
  ):
    fields = [
      ("string", "workpiece_object_name", workpiece_object_name),
      ("string", "parent_object", parent_object),
      ("string", "grasp_frame_name", grasp_frame_name),
      ("string", "pregrasp_frame_name", pregrasp_frame_name),
      ("string", "tool_object_name", self.tool_object_name),
      ("string", "tool_frame_name", self.tool_frame_name),
      ("string", "cad_json", self.cad_json),
      ("double", "pregrasp_distance_m", self.config.mesh_pregrasp_distance_m),
      (
        "double",
        "max_approach_tilt_deg",
        self.config.mesh_max_approach_tilt_deg,
      ),
    ]
    signature = self.solution.proto_builder.create_signature_with_args(
      parameters=pb.MessageSpec(
        fields=[
          pb.FieldSpec(type=kind, name=key, number=i, arg=value)
          for i, (kind, key, value) in enumerate(fields, 1)
        ]
      )
    )
    return bt.Task(
      name=name or "Plan Mesh Grasp (Local Checks)",
      action=bt.PythonScript(
        signature_with_args=signature,
        function_body=runtime_source(),
      ),
    )
