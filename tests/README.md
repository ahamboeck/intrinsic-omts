# Test Suite (`tests/`)

Hermetic unit test suites verifying OMTS domain logic, YAML configuration
validation, SBL Behavior Tree structure, hardware adapters, and CLI tools.

## Running Tests

```bash
# Run all 20 unit test targets offline (no live cluster or hardware required)
bazel test //tests/...

# Run a specific test suite with detailed error output
bazel test --test_output=errors //tests/unit:test_behaviors
```

## Unit Test Coverage (`tests/unit/`)

| Test Target | Module Under Test | Key Verifications |
| :--- | :--- | :--- |
| `test_config` | `src/core/config.py`, `src/grasping/registry.py`, `src/main.py` | Strict YAML loading (`omts`, `lab_bb_01`), missing key rejection, optional `machine` section handling, gripper model and planner section parsing, planner registry validation and selection. |
| `test_behaviors` | `src/behaviors/*.py` | Master BT assembly (`bt.Loop` single/finite/continuous), optional `machine=None` node omission, segment-scoped collision rules, compliant touchdowns, and linear retracts across all 5 subtrees. |
| `test_hardware_adapters` | `src/hardware/*.py` | `Robot`, `RobotiqGripper`, `DioGripper`, `DioCncMachine`, and `OrbbecVision` skill generation, ADIO capability resolution, and belief-world joint synchronization. |
| `test_infeed` | `src/core/infeed.py` | `PerceptionInfeedStrategy` and `GridInfeedStrategy` part acquisition lifecycle. |
| `test_tray` | `src/core/tray.py` | Row-major slot iteration, pitch offset math, and slot occupancy transitions. |
| `test_workpiece` | `src/core/workpiece.py` | Workpiece state machine transitions (`RAW` through `INSPECTED_OK`/`REJECTED`). |
| `test_frame_scripts` | `src/utils/workpiece_pose.py`, `src/utils/random_placement.py`, `src/utils/regrasp_frames.py`, `src/utils/math_utils.py` | 3D camera-to-root `Pose3` composition and `min_safe_z`, return placement shifts, grip-corrected return frames, and kinematics helpers. |
| `test_cuboid_center_planner` | `src/grasping/planners/cuboid_center/` | Short-side grasp alignment, geodesic $SO(3)$ quaternion selection, `list_frames()` update vs. create branching, and the planner's script parameters. |
| `test_antipodal_mesh_plan_grasp` | `src/grasping/planners/antipodal_mesh/`, `src/grasping/workpiece_mesh.py`, `src/grasping/feasibility.py` | Mesh grasp skill checks and frame writes, mesh fetching, and the planner adapter. |
| `test_grasp_candidates`, `test_mesh_workpieces` | `src/grasping/planners/antipodal_mesh/candidates.py`, `src/grasping/workpiece_mesh.py` | Antipodal candidate generation, mesh validation, and grasp candidates on every demo workpiece. |
| `test_gripper_model` | `configs/*/app_config.yaml` | The cells' Hand-E `gripper.model` matches the patched Hand-E SDF colliders. |
| `test_script_utils` | `src/utils/script_utils.py` | Source code extraction for `bt.PythonScript` and `min_safe_z` enforcement. |
| `test_execution_utils` | `src/utils/execution_utils.py` | Mapping between `SimulationMode` and `Executive.SimulationMode`. |
| `test_world_configs` | `configs/**/*.pbtxt`, `configs/**/*.textproto` | Protobuf syntax and schema validation for all cell `.pbtxt` world update files, calibration waypoints, and `.textproto` service configs/manifests. |
| `test_apply_scene_updates` | `tools/world/apply_scene_updates.py`, `tools/world/inspect_world.py` | Live `create_frame` → `update_transform` adaptation, simulator reset handling, and world/resource inspection. |
| `test_control_gripper` | `tools/gripper/control_gripper.py` | CLI argument parsing and gripper open/close task dispatch. |
| `test_control_machine` | `tools/machine/control_machine.py` | CLI argument parsing and CNC door/vise/cycle action execution. |
| `test_move_to_frame` | `tools/jogging/move_to_frame.py` | Frame discovery and Cartesian move execution. |
| `test_move_to_joint` | `tools/jogging/move_to_joint.py`, `tools/jogging/store_joint_config.py`, `tools/jogging/jog_interactive.py` | Joint configuration selection, belief + `init_world` persistence, and interactive jogging helpers. |
| `test_store_frame` | `tools/jogging/store_frame.py` | Capturing live tool transforms and updating `.pbtxt` frame definitions. |
| `test_calibrate_camera` | `tools/calibration/*.py` | Camera calibration skill parameter wiring. |
| `test_sample_calibration_poses` | `tools/calibration/sample_calibration_poses.py` | ICON arm part resolution, camera streaming, and manual waypoint recording. |
| `test_update_robot_kinematics` | `tools/calibration/update_robot_kinematics.py` | Robot kinematic calibration update utilities. |

