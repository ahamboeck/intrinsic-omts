"""Offline Lab BB-01 mesh grasp visualization data; never connects to a robot."""

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

from src.grasping.contact_pairs import propose_contact_pairs
from src.grasping.geometry import RigidTransform
from src.grasping.hande_profile import hande_collision_profile
from src.grasping.local_checks import check_local_grasp
from src.grasping.poses import propose_top_down_grasps
from src.grasping.types import ContactPairOptions, TriangleMesh


def plan_demo(cad: dict, pose: dict, max_tilt_deg: float = 2) -> dict:
  """Use actual CAD and a recorded world object pose, without feasibility claims.

  CAD vertices must already be in object coordinates and metres. Reindex only
  exactly coincident export seams; do not approximate or change mesh geometry.
  Pose rotation is basis COLUMNS, not rows. Never use simulator ground truth.
  """
  vertices, index, remap = [], {}, []
  for point in cad["vertices"]:
    point = tuple(point)
    if point not in index:
      index[point] = len(vertices)
      vertices.append(point)
    remap.append(index[point])
  mesh = TriangleMesh(
    tuple(vertices),
    tuple(tuple(remap[i] for i in face) for face in cad["triangles"]),
  )
  world = RigidTransform(
    tuple(tuple(axis) for axis in pose["rotation_basis_columns"]),
    tuple(pose["translation_m"]),
  )
  profile = hande_collision_profile()
  pairs = propose_contact_pairs(
    mesh,
    profile.limits,
    options=ContactPairOptions(
      seed=42,
      max_pairs=20,
      max_samples=2000,
      opening_clearance_m=0.001,
    ),
  )
  proposals = propose_top_down_grasps(
    tuple(pairs),
    profile,
    world_T_object=world,
    pregrasp_distance_m=0.1,
    open_width_m=0.06,
    max_approach_tilt_rad=math.radians(max_tilt_deg),
  )
  candidates = []
  for proposal in proposals:
    check = check_local_grasp(mesh, proposal, profile)
    candidates.append(
      {
        "local_check": asdict(check),
        "world_T_tcp": asdict(world.compose(proposal.object_T_tcp)),
        "world_T_pregrasp_tcp": asdict(
          world.compose(proposal.object_T_pregrasp_tcp)
        ),
        "contacts": asdict(proposal.contacts),
        "open_width_m": proposal.open_width_m,
      }
    )
  return {
    "mode": "offline_recording_only",
    "robot_scene_feasibility": "NOT_CHECKED",
    "execution_ready": False,
    "contact_pairs": len(pairs),
    "proposals": len(proposals),
    "locally_accepted": sum(c["local_check"]["accepted"] for c in candidates),
    "candidates": candidates,
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--cad", required=True, type=Path)
  parser.add_argument("--world-object-pose", required=True, type=Path)
  parser.add_argument("--max-tilt-deg", type=float, default=2)
  args = parser.parse_args()
  report = plan_demo(
    json.loads(args.cad.read_text()),
    json.loads(args.world_object_pose.read_text()),
    args.max_tilt_deg,
  )
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
