"""Pure world-horizontal camera-orbit and evaluation math (metres, basis columns)."""

import hashlib
import json
import math

from src.grasping.geometry import RigidTransform, cross, dot, sub, unit


def record(pose):
  return dict(
    translation_m=pose.translation, rotation_basis_columns=pose.rotation
  )


def transform(value):
  return RigidTransform(
    tuple(map(tuple, value["rotation_basis_columns"])),
    tuple(value["translation_m"]),
  )


def plan_digest(plan):
  return hashlib.sha256(
    json.dumps(plan, sort_keys=True, allow_nan=False).encode()
  ).hexdigest()


def pair_results(baseline, corrected):
  """Join single-object samples by plan/view/repeat, never by ground-truth error.

  Multi-detection samples remain explicitly ambiguous; do not cherry-pick the
  detection nearest simulator truth. Missing samples also remain in the table.
  """
  grouped = [{}, {}]
  for rows, groups in zip((baseline, corrected), grouped, strict=True):
    for row in rows:
      key = (row["plan_id"], row["view_id"], row["repeat"])
      groups.setdefault(key, []).append(row)
  plan_ids = {key[0] for groups in grouped for key in groups}
  if len(plan_ids) != 1:
    raise ValueError("Both runs must use the same saved plan.")
  output = []
  for key in sorted(grouped[0].keys() | grouped[1].keys()):
    row = dict(zip(("plan_id", "view_id", "repeat"), key, strict=True))
    for label, groups in zip(("baseline", "corrected"), grouped, strict=True):
      samples = groups.get(key, [])
      if len(samples) == 1:
        row.update(
          {f"{label}_{k}": v for k, v in samples[0].items() if k not in row}
        )
      else:
        row[f"{label}_status"] = (
          "missing" if not samples else "ambiguous_multiple_rows"
        )
    output.append(row)
  return output


def make_plan(actual, camera, count=12, radius=None, height=None):
  """Aim optical +Z at center, with optical -Y approximately world up.

  The center is simulator evaluation state, explicitly allowed ONLY for this
  benchmark's view acquisition. No object orientation enters camera targeting.
  Default radius, elevation and starting azimuth match the current optical view.
  Targets are not a promise of reachability, collision freedom or visibility.
  """
  delta = sub(camera.translation, actual.translation)
  radius = math.hypot(*delta[:2]) if radius is None else radius
  height = delta[2] if height is None else height
  if not isinstance(count, int) or isinstance(count, bool) or count < 1:
    raise ValueError("count must be a positive integer.")
  if not all(math.isfinite(x) and x > 0 for x in (radius, height)):
    raise ValueError("radius and height must be positive finite metres.")
  start = math.atan2(delta[1], delta[0])
  views = []
  for index in range(count):
    angle = start + 2 * math.pi * index / count
    position = (
      actual.translation[0] + radius * math.cos(angle),
      actual.translation[1] + radius * math.sin(angle),
      actual.translation[2] + height,
    )
    z_axis = unit(sub(actual.translation, position))
    x_axis = unit(cross(z_axis, (0.0, 0.0, 1.0)))
    y_axis = cross(z_axis, x_axis)
    views.append(
      dict(
        view_id=f"view_{index:03d}",
        azimuth_deg=math.degrees(angle),
        camera_pose=record(RigidTransform((x_axis, y_axis, z_axis), position)),
      )
    )
  return dict(
    schema_version=1,
    center_pose=record(actual),
    radius_m=radius,
    height_above_center_m=height,
    views=views,
    ground_truth_use="benchmark acquisition path and evaluation only",
  )


def pose_errors(detected, actual):
  """Raw SO(3) error and separately labelled rectangular-cuboid sign symmetry.

  Distinct CAD dimensions prohibit axis permutations. Four proper diagonal
  sign flips represent a featureless cuboid's 180-degree symmetries, not a
  claim that textured CAD has those symmetries.
  """
  delta = sub(detected.translation, actual.translation)
  diagonal = tuple(
    dot(a, b) for a, b in zip(detected.rotation, actual.rotation, strict=True)
  )

  def angle(cosine):
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

  return dict(
    position_error_mm=math.hypot(*delta) * 1000,
    rotation_error_deg=angle((sum(diagonal) - 1) / 2),
    cuboid_symmetry_rotation_error_deg=min(
      angle((sum(d * s for d, s in zip(diagonal, signs, strict=True)) - 1) / 2)
      for signs in ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1))
    ),
    **{
      f"delta_{axis}_mm": value * 1000
      for axis, value in zip("xyz", delta, strict=True)
    },
    **{
      f"unsigned_{axis}_axis_error_deg": angle(abs(value))
      for axis, value in zip("xyz", diagonal, strict=True)
    },
  )
