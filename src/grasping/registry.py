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

"""Maps the planner name in `grasp.planner` to its adapter.

To add a planner, add its folder under `planners/`, an entry to `PLANNERS`, and
select it with `grasp: {planner: "<name>"}`. Its settings come from the
`grasp.<name>` config section.
"""

import dataclasses
from collections.abc import Callable, Mapping
from typing import Any

from src.core.config import AppConfig, GraspConfig
from src.grasping.interface import GraspPlannerInterface, NoSettings
from src.grasping.planners.antipodal_mesh import adapter as antipodal_mesh
from src.grasping.planners.cuboid_center import adapter as cuboid_center
from src.grasping.planners.moveit import adapter as moveit


@dataclasses.dataclass(frozen=True)
class PlannerEntry:
  """How to build one registered planner.

  Attributes:
      create: Builds the planner from the connected solution, the cell config
        and the parsed settings.
      settings: Frozen dataclass parsed from the planner's config section. Its
        fields need defaults, since the section may be omitted.
  """

  create: Callable[[Any, AppConfig, Any], GraspPlannerInterface]
  settings: type = NoSettings


PLANNERS: Mapping[str, PlannerEntry] = {
  "antipodal_mesh": PlannerEntry(
    antipodal_mesh.create, antipodal_mesh.AntipodalMeshSettings
  ),
  "cuboid_center": PlannerEntry(cuboid_center.create),
  "moveit": PlannerEntry(moveit.create, moveit.MoveItSettings),
}


def _entry(name: str) -> PlannerEntry:
  if name not in PLANNERS:
    raise ValueError(
      f"Unsupported grasp planner '{name}' in section 'grasp'. Supported: "
      f"{sorted(PLANNERS)}."
    )
  return PLANNERS[name]


def planner_settings(grasp: GraspConfig, name: str) -> Any:
  """Returns the parsed `grasp.<name>` section, with defaults if omitted.

  Raises:
      ValueError: If `name` is not registered, or the section has unknown or
        invalid fields.
  """
  settings_cls = _entry(name).settings
  raw = dict(grasp.settings.get(name) or {})
  known = {field.name for field in dataclasses.fields(settings_cls)}
  unknown = sorted(set(raw) - known)
  if unknown:
    raise ValueError(
      f"Unknown field(s) {unknown} in section 'grasp.{name}'. Supported: "
      f"{sorted(known)}."
    )
  return settings_cls(**raw)


def gripper_kind(config: AppConfig) -> str | None:
  """Returns the kind of `gripper.model`, or None if the cell has no model."""
  model = config.gripper.model
  return model.WhichOneof("kind") if model is not None else None


def validate(grasp: GraspConfig, planner: str | None = None) -> None:
  """Checks the selected planner and every planner section in `grasp`.

  Args:
      grasp: Grasp section of the cell config.
      planner: Planner selected for this run, `grasp.planner` by default.

  Raises:
      ValueError: If a planner name is not registered, or a section is invalid.
  """
  _entry(planner or grasp.planner)
  for name in grasp.settings:
    planner_settings(grasp, name)


def create_grasp_planner(
  solution: Any, config: AppConfig, planner: str | None = None
) -> GraspPlannerInterface:
  """Builds the grasp planner for this run.

  Args:
      solution: Connected solution deployment.
      config: Cell config. Its `grasp` section selects and tunes the planner.
      planner: Planner name overriding `config.grasp.planner`.

  Returns:
      The planner.

  Raises:
      ValueError: If the planner or its settings are invalid, the cell's
        gripper kind is not supported by the planner, the planner is
        simulation only without the `grasp.simulation_only` acknowledgement,
        or the planner's skill is missing from the solution.
  """
  name = planner or config.grasp.planner
  validate(config.grasp, name)
  grasp_planner = _entry(name).create(
    solution, config, planner_settings(config.grasp, name)
  )
  kind = gripper_kind(config)
  if kind is not None and kind not in grasp_planner.gripper_kinds:
    raise ValueError(
      f"Grasp planner '{name}' does not support the cell's '{kind}' gripper. "
      f"Supported: {sorted(grasp_planner.gripper_kinds)}."
    )
  if grasp_planner.simulation_only and not config.grasp.simulation_only:
    raise ValueError(
      f"Grasp planner '{name}' is verified in simulation only. Set "
      "`grasp: {simulation_only: true}` or pass --grasp_simulation_only to "
      "acknowledge that this deployment is simulated."
    )
  return grasp_planner
