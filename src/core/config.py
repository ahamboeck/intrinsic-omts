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

"""Cell configuration dataclasses and strict YAML loader for OMTS workcells."""

import dataclasses
import pathlib
from collections.abc import Mapping
from typing import Any, TypeVar

import yaml
from google.protobuf import json_format

from src.core.types import Touchdown
from src.grasping import gripper_model_pb2

_T = TypeVar("_T")


@dataclasses.dataclass(frozen=True)
class RobotConfig:
  """Configuration for the robot arm and moving tool frame.

  Attributes:
      arm_part_name: Name of the robot arm part in `solution.world`.
      tool_object_name: Name of the end-effector object in `solution.world`.
      tool_frame_name: Name of the TCP frame on `tool_object_name`.
      enclosure_object_name: Name of the cell enclosure object, if applicable.
  """

  arm_part_name: str
  tool_object_name: str
  tool_frame_name: str
  enclosure_object_name: str | None = None


@dataclasses.dataclass(frozen=True)
class GripperType:
  """What the cell config needs to know about one `gripper.type`.

  Attributes:
      kind: The `gripper.model` kind, a field of `GripperModel`, that grippers
        of this type are described by.
      required_fields: `GripperConfig` fields this type's adapter needs.
  """

  kind: str
  required_fields: tuple[str, ...]


# The gripper types a cell config may use. Each has an adapter of the same name
# in `src.hardware.gripper.GRIPPERS`. To add a gripper type, add it here and
# there.
GRIPPER_TYPES: Mapping[str, GripperType] = {
  "dio": GripperType(
    kind="parallel_jaw",
    required_fields=("dio_open_pin", "dio_close_pin", "dio_output_block_name"),
  ),
  "robotiq": GripperType(
    kind="parallel_jaw",
    required_fields=("joint_name", "open_position", "close_position"),
  ),
  "suction": GripperType(kind="suction", required_fields=("resource_name",)),
}


@dataclasses.dataclass(frozen=True)
class GripperConfig:
  """Configuration for the end-effector gripper adapter.

  Attributes:
      type: Gripper adapter type, a key of `GRIPPER_TYPES`.
      joint_name: Finger joint name for `RobotiqGripper`.
      open_position: Open stroke position in meters for `RobotiqGripper`.
      close_position: Closed stroke position in meters for `RobotiqGripper`.
      action_name: Optional action resource name for `RobotiqGripper`.
      dio_open_pin: Digital output index to open `DioGripper`.
      dio_close_pin: Digital output index to close `DioGripper`.
      dio_device_name: Optional ADIO device resource name for `DioGripper`.
      dio_output_block_name: Digital output block name for `DioGripper`.
      resource_name: Resource of the EOAT suction gripper for
        `SuctionGripper`, such as the hardware device instance `'gripper'`.
      model: Gripper geometry that grasp planners plan with, such as the jaw
        widths of a parallel-jaw gripper. Optional for planners that need no
        geometry.
  """

  type: str
  joint_name: str | None = None
  open_position: float | None = None
  close_position: float | None = None
  action_name: str | None = None
  dio_open_pin: int | None = None
  dio_close_pin: int | None = None
  dio_device_name: str | None = None
  dio_output_block_name: str | None = None
  resource_name: str | None = None
  model: gripper_model_pb2.GripperModel | None = None

  def __post_init__(self) -> None:
    gripper_type = GRIPPER_TYPES.get(self.type)
    if gripper_type is None:
      raise ValueError(
        f"Unsupported gripper type '{self.type}'. Supported: "
        f"{sorted(GRIPPER_TYPES)}."
      )
    missing = [
      name
      for name in gripper_type.required_fields
      if getattr(self, name) is None
    ]
    if missing:
      raise KeyError(
        f"Missing required configuration field(s) {sorted(missing)} for "
        f"gripper type '{self.type}'"
      )
    kind = self.model.WhichOneof("kind") if self.model is not None else None
    if kind is not None and kind != gripper_type.kind:
      raise ValueError(
        f"Gripper type '{self.type}' is described by a '{gripper_type.kind}' "
        f"model, but 'gripper.model' is '{kind}'."
      )


@dataclasses.dataclass(frozen=True)
class MachineConfig:
  """Configuration for CNC enclosure door, vise, and cycle handshake DIO.

  Attributes:
      door_open_pin: Digital output index to open the CNC door.
      door_close_pin: Digital output index to close the CNC door.
      vise_open_pin: Digital output index to open the pneumatic vise.
      vise_close_pin: Digital output index to clamp the pneumatic vise.
      cycle_start_pin: Digital output index pulsed to trigger cycle start.
      cycle_complete_input_pin: Optional digital input index for cycle done.
      device_name: ADIO device resource name (e.g. `'ur_module'`).
      enclosure_object_name: Scene object name for the CNC enclosure door.
      vise_object_name: Scene object name for the pneumatic vise.
      door_open_joints: Joint position vector for open enclosure door.
      door_closed_joints: Joint position vector for closed enclosure door.
      vise_open_joints: Joint position vector for open vise jaws.
      vise_closed_joints: Joint position vector for clamped vise jaws.
      output_block_name: Digital output block name on the ADIO device.
      input_block_name: Digital input block name on the ADIO device.
  """

  door_open_pin: int
  door_close_pin: int
  vise_open_pin: int
  vise_close_pin: int
  cycle_start_pin: int
  cycle_complete_input_pin: int | None
  device_name: str
  enclosure_object_name: str | None
  vise_object_name: str | None
  door_open_joints: tuple[float, ...]
  door_closed_joints: tuple[float, ...]
  vise_open_joints: tuple[float, ...]
  vise_closed_joints: tuple[float, ...]
  output_block_name: str
  input_block_name: str


@dataclasses.dataclass(frozen=True)
class VisionConfig:
  """Configuration for 3D perception and pose estimation.

  Attributes:
      camera_name: Resource name of the RGB-D camera.
      perception_service_name: Resource name of the pose estimation service.
      pose_estimator_id: Asset ID of the registered pose estimator.
      scene_object_id: Scene object ID updated upon pose detection.
      sensor_ids: Camera sensor stream IDs (e.g. `(1, 4)` for RGB-D).
      min_num_instances: Minimum detected part instances required per capture.
      infeed_mode: Infeed localization strategy (`'perception'` or `'grid'`).
      min_safe_z: Minimum allowable Z coordinate in `root` frame (meters).
  """

  camera_name: str
  perception_service_name: str
  pose_estimator_id: str
  scene_object_id: str
  sensor_ids: tuple[int, ...]
  min_num_instances: int
  infeed_mode: str
  min_safe_z: float


@dataclasses.dataclass(frozen=True)
class FramesConfig:
  """World transform frame names used across machine tending motions.

  Attributes:
      parent_object: World object owning the scene frames (e.g. `'root'`).
      view_frame: Camera observation pose frame name.
      pregrasp_frame: Pre-grasp approach frame name above the workpiece.
      grasp_frame: Target grasp frame name on the workpiece.
      machine_approach_frame: Entry/standby frame outside the CNC enclosure.
      preplace_vise_frame: Pre-placement approach frame above the CNC vise.
      place_vise_frame: Seated part frame inside the CNC vise.
      transit_frame: Optional intermediate waypoint frame for blended transits.
  """

  parent_object: str
  view_frame: str
  pregrasp_frame: str
  grasp_frame: str
  machine_approach_frame: str
  preplace_vise_frame: str
  place_vise_frame: str
  transit_frame: str | None = None


@dataclasses.dataclass(frozen=True)
class CycleConfig:
  """Execution, force, and motion parameters for the machine tending cycle.

  Attributes:
      num_cycles: Number of cycles to run (`1` = single, `>1` = finite loop,
        `<=0` = continuous loop).
      workpiece_id: Name of the workpiece object in `solution.world`.
      approach_offset_z: Vertical offset in meters from `grasp` to `pre_grasp`.
      standoff_distance_meters: Linear approach standoff distance in meters
        prior to compliant `move_to_contact` touchdown.
      retract_distance_meters: Tool `-Z` linear retract distance in meters.
      pick_touchdown_force_newtons: Force threshold for infeed pick contact (N).
      load_seat_force_newtons: Force threshold for seating part in vise (N).
      unload_touchdown_force_newtons: Force threshold for unload contact (N).
      return_touchdown_force_newtons: Force threshold for table placement (N).
      touchdown_timeout_seconds: Timeout for `move_to_contact` actions (s).
      machining_timeout_seconds: Maximum duration to wait for CNC cycle (s).
      return_shift: Optional configuration for applying a randomized positional
        shift when returning the workpiece. If None, the object is placed at
        the exact view frame location.
  """

  num_cycles: int
  workpiece_id: str
  approach_offset_z: float
  standoff_distance_meters: float
  retract_distance_meters: float
  pick_touchdown_force_newtons: float
  load_seat_force_newtons: float
  unload_touchdown_force_newtons: float
  return_touchdown_force_newtons: float
  touchdown_timeout_seconds: float
  machining_timeout_seconds: float
  return_shift: "ReturnShiftConfig | None" = None


@dataclasses.dataclass(frozen=True)
class ReturnShiftConfig:
  """Parameters for randomizing the placement position of the returned workpiece."""

  center_x: float
  center_y: float
  bounds_x: float
  bounds_y: float
  bounds_rz_degrees: float = 0.0


@dataclasses.dataclass(frozen=True)
class GraspConfig:
  """Selects and tunes the grasp planner.

  In YAML, every key of the `grasp` section other than `planner` and
  `simulation_only` is a planner's own section, such as `grasp.moveit`. The
  planner registry in `src/grasping/registry.py` validates the planner name
  and parses those sections.

  Attributes:
      planner: Name of the registered grasp planner.
      simulation_only: Acknowledges that the deployment is simulated, as
        required by planners that are not verified on hardware.
      settings: Planner name to its config section, as loaded from YAML.
  """

  planner: str = "cuboid_center"
  simulation_only: bool = False
  settings: Mapping[str, Mapping[str, Any]] = dataclasses.field(
    default_factory=dict
  )


@dataclasses.dataclass(frozen=True)
class AppConfig:
  """Top-level configuration for a machine tending cell deployment.

  Attributes:
      cell_name: Identifier of the configured workcell (e.g. `'omts'`).
      robot: Robot arm and tool frame configuration.
      gripper: End-effector gripper configuration.
      vision: 3D perception and pose estimation configuration.
      frames: Named world transform frames for motion planning.
      cycle: Cycle execution, force, and timeout parameters.
      machine: Optional CNC enclosure, vise, and handshake configuration.
      grasp: Grasp planner selection. Defaults to the built-in cuboid-center
        planner when the section is omitted.
  """

  cell_name: str
  robot: RobotConfig
  gripper: GripperConfig
  vision: VisionConfig
  frames: FramesConfig
  cycle: CycleConfig
  machine: MachineConfig | None = None
  grasp: GraspConfig = dataclasses.field(default_factory=GraspConfig)

  @property
  def pick_touchdown(self) -> Touchdown:
    """Returns compliant touchdown parameters for picking from the infeed."""
    return Touchdown(
      force_n=self.cycle.pick_touchdown_force_newtons,
      standoff_m=self.cycle.standoff_distance_meters,
      timeout_s=self.cycle.touchdown_timeout_seconds,
      retract_after_m=self.cycle.retract_distance_meters,
    )

  @property
  def load_touchdown(self) -> Touchdown:
    """Returns compliant touchdown parameters for seating into the CNC vise."""
    return Touchdown(
      force_n=self.cycle.load_seat_force_newtons,
      standoff_m=self.cycle.standoff_distance_meters,
      timeout_s=self.cycle.touchdown_timeout_seconds,
      retract_after_m=0.0,
    )

  @property
  def unload_touchdown(self) -> Touchdown:
    """Returns compliant touchdown parameters for grasping from the CNC vise."""
    return Touchdown(
      force_n=self.cycle.unload_touchdown_force_newtons,
      standoff_m=(
        self.cycle.standoff_distance_meters + self.cycle.retract_distance_meters
      ),
      timeout_s=self.cycle.touchdown_timeout_seconds,
      retract_after_m=self.cycle.retract_distance_meters,
    )

  @property
  def return_touchdown(self) -> Touchdown:
    """Returns compliant touchdown parameters for returning to the infeed."""
    return Touchdown(
      force_n=self.cycle.return_touchdown_force_newtons,
      standoff_m=(
        self.cycle.standoff_distance_meters + self.cycle.retract_distance_meters
      ),
      timeout_s=self.cycle.touchdown_timeout_seconds,
      retract_after_m=0.0,
    )


def _construct_section(
  cls: type[_T],
  raw_data: dict[str, Any],
  section_name: str,
  file_path: pathlib.Path,
) -> _T:
  """Validates that all fields of `cls` exist in `raw_data[section_name]` and constructs it."""
  if section_name not in raw_data or not isinstance(
    raw_data[section_name], dict
  ):
    raise KeyError(
      f"Missing required configuration section '{section_name}' in {file_path}"
    )

  section_dict = dict(raw_data[section_name])
  # The gripper type's own required fields are checked by `GripperConfig`.
  required_fields = {
    f.name
    for f in dataclasses.fields(cls)  # type: ignore[arg-type]
    if f.default is dataclasses.MISSING
    and f.default_factory is dataclasses.MISSING
  }
  missing_fields = required_fields - set(section_dict.keys())
  if missing_fields:
    sorted_missing = sorted(missing_fields)
    raise KeyError(
      f"Missing required configuration field(s) {sorted_missing} in section "
      f"'{section_name}' of {file_path}"
    )

  if section_name == "vision" and isinstance(
    section_dict.get("sensor_ids"), list
  ):
    section_dict["sensor_ids"] = tuple(
      int(x) for x in section_dict["sensor_ids"]
    )
  elif section_name == "cycle" and section_dict.get("return_shift") is not None:
    section_dict["return_shift"] = ReturnShiftConfig(
      **section_dict["return_shift"]
    )
  elif section_name == "machine":
    for joint_key in (
      "door_open_joints",
      "door_closed_joints",
      "vise_open_joints",
      "vise_closed_joints",
    ):
      if isinstance(section_dict.get(joint_key), list):
        section_dict[joint_key] = tuple(
          float(x) for x in section_dict[joint_key]
        )
  elif section_name == "gripper" and section_dict.get("model") is not None:
    section_dict["model"] = _parse_gripper_model(
      section_dict["model"], file_path
    )
  elif section_name == "grasp":
    planner = section_dict.pop("planner", GraspConfig.planner)
    simulation_only = section_dict.pop("simulation_only", False)
    for name, settings in section_dict.items():
      if settings is not None and not isinstance(settings, dict):
        raise ValueError(
          f"Section 'grasp.{name}' in {file_path} must be a mapping of "
          f"planner settings."
        )
    return cls(
      planner=planner,
      simulation_only=bool(simulation_only),
      settings={
        name: dict(value or {}) for name, value in section_dict.items()
      },
    )

  return cls(**section_dict)


def _parse_gripper_model(
  raw: Any, file_path: pathlib.Path
) -> gripper_model_pb2.GripperModel:
  """Parses `gripper.model`, which must name exactly one gripper kind."""
  try:
    model = json_format.ParseDict(raw, gripper_model_pb2.GripperModel())
  except json_format.ParseError as exc:
    raise ValueError(f"Invalid 'gripper.model' in {file_path}: {exc}") from exc
  if model.WhichOneof("kind") is None:
    raise ValueError(
      f"'gripper.model' in {file_path} must name a gripper kind, such as "
      "'parallel_jaw'."
    )
  return model


def load_app_config(path: str | pathlib.Path) -> AppConfig:
  """Loads and strictly validates an AppConfig from a YAML or JSON configuration file.

  Fails loudly with KeyError if any required section or field is omitted.

  Args:
      path: File path to the YAML configuration file.

  Returns:
      Populated AppConfig dataclass instance.
  """
  file_path = pathlib.Path(path)
  content = file_path.read_text(encoding="utf-8")
  raw_data = yaml.safe_load(content)

  if not isinstance(raw_data, dict):
    raise ValueError(
      f"Configuration file {file_path} must contain a top-level mapping."
    )
  if "cell_name" not in raw_data or not raw_data["cell_name"]:
    raise KeyError(f"Missing required field 'cell_name' in {file_path}")

  machine_config = (
    _construct_section(MachineConfig, raw_data, "machine", file_path)
    if "machine" in raw_data and raw_data["machine"] is not None
    else None
  )
  grasp_config = (
    _construct_section(GraspConfig, raw_data, "grasp", file_path)
    if "grasp" in raw_data and raw_data["grasp"] is not None
    else GraspConfig()
  )

  return AppConfig(
    cell_name=str(raw_data["cell_name"]),
    robot=_construct_section(RobotConfig, raw_data, "robot", file_path),
    gripper=_construct_section(GripperConfig, raw_data, "gripper", file_path),
    machine=machine_config,
    vision=_construct_section(VisionConfig, raw_data, "vision", file_path),
    frames=_construct_section(FramesConfig, raw_data, "frames", file_path),
    cycle=_construct_section(CycleConfig, raw_data, "cycle", file_path),
    grasp=grasp_config,
  )


def with_workpiece(config: AppConfig, workpiece_id: str) -> AppConfig:
  """Returns `config` with another workpiece in the same cell.

  The workpiece's scene object and text-prompted pose estimator are named as
  for `raw_stock_2x3x5`: `ai.intrinsic.<workpiece_id>` and
  `ai.intrinsic.<workpiece_id>_estimator_text`.
  """
  return dataclasses.replace(
    config,
    vision=dataclasses.replace(
      config.vision,
      pose_estimator_id=f"ai.intrinsic.{workpiece_id}_estimator_text",
      scene_object_id=f"ai.intrinsic.{workpiece_id}",
    ),
    cycle=dataclasses.replace(config.cycle, workpiece_id=workpiece_id),
  )
