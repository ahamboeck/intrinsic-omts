# Hardware Abstraction Layer (`src/hardware/`)

Stateless hardware adapters that translate high-level robotic manipulation,
gripping, machine tool I/O, and 3D perception actions into Intrinsic SBL skill
nodes (`bt.Task` / `bt.Sequence`).

## Modules & Adapters

| Module | Abstract Interface | Concrete Implementation(s) | SBL Skills Wrapped |
| :--- | :--- | :--- | :--- |
| [`robot.py`](robot.py) | — | `Robot` | `move_robot` (`JOINT`, `LINEAR`, `ANY`, `RelativePoseEquality`, `RotationCone`), `move_to_contact`, `attach_object_to_robot`, `detach_object` |
| [`gripper.py`](gripper.py) | `GripperInterface` | `RobotiqGripper`, `DioGripper` | `gripper_cmd_skill` (metric finger joint position), `dio_set_output` (solenoid/relay pins) |
| [`machine.py`](machine.py) | `CncMachineInterface` | `DioCncMachine` | `dio_set_output`, `dio_wait_for_input` / `dio_read_input`, `update_world` (belief-world door/vise joint synchronization) |
| [`vision.py`](vision.py) | `VisionInterface` | `OrbbecVision` | `capture_images`, `estimate_pose_multi_view` (FoundationPose), `bt.PythonScript` workpiece pose update wrapped in `bt.Retry` |

Grasp planners are not hardware adapters; they live in
[`src/grasping/`](../grasping/README.md).

## Adapter Design Rules

* **No Robot Interface**: `move_robot`, `move_to_contact`,
  `attach_object_to_robot`, and `detach_object` are vendor-neutral and bind to
  any ICON-controlled arm, so a single `Robot` class serves UR, KUKA, FANUC,
  etc. Robot-specific details come from the world (kinematics, named joint
  configurations) and `RobotConfig` (arm part and tool frame names).
* **Scoped Configuration Dataclasses**: Every adapter constructor accepts the
  connected `solution` handle and its scoped configuration dataclass
  (`RobotConfig`, `GripperConfig`, `MachineConfig`, `VisionConfig`).
* **Gripper Types**: `gripper.type` in the cell config selects the adapter.
  `GRIPPER_TYPES` in [`src/core/config.py`](../core/config.py) lists each type's
  `gripper.model` kind and required fields, which are checked when the config
  loads, and `GRIPPERS` in [`gripper.py`](gripper.py) maps the same types to
  their adapter classes; `create_gripper()` builds the cell's gripper. To add a
  gripper type, add an adapter implementing `GripperInterface` and an entry to
  both tables.
* **Capability-Filtered ADIO Binding**: `DioGripper` and `DioCncMachine` resolve
  optional ADIO device handles via `resolve_adio_resource()` so skills only bind
  explicit resources advertising `Icon2AdioPart`.
* **Belief-World Joint Synchronization**: When `DioCncMachine` commands door or
  vise digital outputs, it appends an `update_world` task updating the
  corresponding scene object joint positions (`cnc_enclosure`,
  `schunk_egp_64nnb`) so collision checking reflects the physical state.
* **Perception Only Localizes**: `OrbbecVision.build_perception_and_spawn_task`
  updates the workpiece pose and enforces `min_safe_z`. The `grasp` /
  `pre_grasp` frames belong to the selected grasp planner.
