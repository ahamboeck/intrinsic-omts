# Grasp planning (`src/grasping/`)

A grasp planner decides where the tool grasps the workpiece. Perception only
updates the workpiece's pose; the selected planner then writes the `grasp` and
`pre_grasp` frames from that pose, or fails with a clear error. The pick,
unload and return behaviours own the motion sequence and read from the planner
how to use its grasps.

| File | Contents |
| :--- | :--- |
| [`interface.py`](interface.py) | `GraspPlannerInterface` and `GraspExecution`, the contract between planners and behaviours. |
| [`registry.py`](registry.py) | `PLANNERS`, which maps `grasp.planner` to a planner, and `create_grasp_planner`. |
| [`gripper_model.proto`](gripper_model.proto) | Gripper geometry per cell (`gripper.model`), such as a parallel-jaw gripper's widths. |
| [`workpiece_mesh.py`](workpiece_mesh.py) | Fetches and validates a workpiece's triangle mesh from the Object World. |
| [`feasibility.py`](feasibility.py) | Checks a grasp with Core's motion planner: collision-checked IK at the pregrasp, then a collision-checked linear path to the grasp. |
| [`planners/cuboid_center/`](planners/cuboid_center/) | The default: a top-down grasp at the center of the part's pose, with a touchdown. |
| [`planners/moveit/`](planners/moveit/) | Registers the [MoveIt integration](../../third_party/intrinsic_moveit/README.md). |
| [`planners/antipodal_mesh/`](planners/antipodal_mesh/) | Plans on two opposing contacts of the part's mesh. Simulation only. |

## The planner contract

A planner implements `build_plan_grasp_task`, which returns a behavior tree
node that writes the two frames. It can be a skill, an inline `PythonScript` or
MoveIt. It also sets three class attributes:

- `execution`, a `GraspExecution`, tells the behaviours how to use its grasps:

  | Field | Effect |
  | :--- | :--- |
  | `open_before_planning` | Opens the gripper before planning, for planners that check the open jaws against the part. Otherwise it opens after planning. |
  | `approach` | `TOUCHDOWN` seats the tool with a force-controlled touchdown and a short retract before closing. `DIRECT` moves linearly onto the grasp and closes there. |
  | `replan_at_unload` | Plans a new grasp on the part in the machine instead of touching down at the vise frame. The return then places the part from its pose saved at the pick and the current grip. |

- `gripper_kinds`: the kinds of `gripper.model` it supports, `parallel_jaw` by
  default.
- `simulation_only`: True until the planner is verified on hardware. Selecting
  it then requires `grasp: {simulation_only: true}` or
  `--grasp_simulation_only`.

| Planner | `open_before_planning` | `approach` | `replan_at_unload` | Simulation only |
| :--- | :--- | :--- | :--- | :--- |
| `cuboid_center` | no | `TOUCHDOWN` | no | no |
| `moveit` | no | `TOUCHDOWN` | no | no |
| `antipodal_mesh` | yes | `DIRECT` | yes | yes |

With `replan_at_unload`, the unload plans into `grasp_unload` and
`pre_grasp_unload`, so the infeed frames stay for the return. The pick saves
the part's pose relative to its grasp in `grasp_workpiece`, a child of `grasp`
that follows the random return shift. Before the return, the tool targets
become `world_T_tool = world_T_grasp_workpiece * part_T_tool` with the current
grip, in `grasp_return` and `pre_grasp_return`. The return then uses the plain
cycle standoff, since these targets are exact. The names follow the configured
grasp frame names and are owned by
[`src/behaviors/regrasp.py`](../behaviors/regrasp.py); planners never see them.

## Configuration

```yaml
gripper:
  type: "robotiq"
  # ...
  model:
    parallel_jaw:
      min_width_m: 0.01
      max_width_m: 0.06
      contact_depth_m: 0.022

grasp:
  planner: "antipodal_mesh"   # default "cuboid_center"
  simulation_only: true       # required by simulation-only planners
  antipodal_mesh:             # this planner's own settings, all optional
    pregrasp_distance_m: 0.1
    max_approach_tilt_deg: 10.0
  moveit:                     # read only when the planner is "moveit"
    surfaces: [0, 1, 4, 5]
```

Every key of `grasp` other than `planner` and `simulation_only` is a planner's
own section, validated at startup, before connecting to the solution. Select a
planner for one run with `--grasp_planner=<name>`.

The Hand-E numbers in both cells' configs are checked against the patched
Hand-E SDF by `tests/unit/test_gripper_model.py`. Another parallel-jaw gripper
only needs its own numbers here, not a new planner.

## Adding a planner

1. Add `planners/<name>/` with an `adapter.py` that implements
   `GraspPlannerInterface`, sets `execution`, `gripper_kinds` and
   `simulation_only`, and provides `create(solution, config, settings)`. Put
   the algorithm next to it, and a thin skill in `skill/` if it needs the
   motion planner or other services, like `antipodal_mesh`.
2. If it has settings, add a frozen dataclass with defaults; it is read from
   `grasp.<name>`.
3. Add an entry to `PLANNERS` in [`registry.py`](registry.py), and its skill,
   if any, to `_OMTS_SKILL_ASSETS` in the root `BUILD`.
4. Select it with `grasp: {planner: "<name>"}`.

Reuse `workpiece_mesh.py` for the part's mesh and `feasibility.py` to check
grasps. The behaviours only change if the planner needs a new kind of
`Approach`, which is then available to every planner.

## The antipodal mesh planner

`antipodal_mesh` picks a parallel-jaw grasp from the workpiece's triangle mesh
instead of its bounding box, so curved and non-cuboid parts can be picked. It
is named after its assumption of two opposing (antipodal) jaw contacts, so
planners for other gripper kinds on a mesh, such as suction, can sit next to
it. It is **simulation only**: the gripper model comes from the Hand-E SDF
colliders and is not calibrated against hardware.

After perception updates the workpiece pose, the pick opens the gripper and
runs the [`antipodal_mesh_plan_grasp`](planners/antipodal_mesh/skill/) skill,
which:

1. Generates top-down candidates
   ([`candidates.py`](planners/antipodal_mesh/candidates.py)): it samples
   points on the near-vertical faces of the mesh, casts a ray from each point
   through the part to the opposite contact, and keeps pairs that fit the jaw
   opening in `gripper.model.parallel_jaw`, whose normals oppose each other
   within 15 degrees, and whose closing fingers do not cross another part of
   the mesh. Candidates closest to the part's centroid come first.
2. Checks candidates in that order with
   [`feasibility.py`](feasibility.py), using Core's own collision models of the
   gripper, the part, the table and the enclosure.
3. Writes the first candidate that passes to the `grasp` and `pre_grasp`
   frames, creating missing frames on `root`. If none passes, the skill fails.

The pick then moves linearly to the grasp pose instead of touching down with
force control first, closes the gripper and lifts. The unload plans again on
the part where the robot placed it, and the return corrects for the new grip
as described above.

The skill plans on the workpiece's visual mesh, the `.glb` its SDF names, which
[`workpiece_mesh.py`](workpiece_mesh.py) reads from the Object World. That mesh
must be closed and outward facing, with at most 10,000 triangles. Validation
rejects open edges, inconsistent winding, nonmanifold edges and degenerate
triangles; duplicated seam vertices are matched by their coordinates.

## Demo workpieces (Lab BB-01)

The lab cell runs `raw_stock_2x3x5` (the default), `banana` or `bracket`. The
demo parts are placed by `configs/lab_bb_01_objects/`. Perception finds the
part in the image from a text prompt, then fits its mesh, from the scene
object `ai.intrinsic.$WORKPIECE`, to get the pose. Registering saves the
prompt and the mesh as the pose estimator
`ai.intrinsic.${WORKPIECE}_estimator_text`, which the app uses:

| `WORKPIECE` | `TEXT_PROMPT` |
| :--- | :--- |
| `banana` | `yellow banana` |
| `bracket` | `U-shaped metal bracket` |

Set both variables once, then run the steps below in the same shell.

1. Choose the workpiece:

   ```bash
   WORKPIECE=banana
   TEXT_PROMPT="yellow banana"
   ```

2. Deploy the lab cell with the workpiece in simulation:

   ```bash
   bazel run //:omts_solution -c opt \
     --//:setup=lab_bb_01 --//:workpiece="$WORKPIECE" -- \
     --address=localhost:17080 \
     --operation_mode=sim
   ```

3. Register the workpiece's pose estimator. This needs the pose estimator
   models from section 4.1 of the top-level README. Repeat it after every
   deploy.

   ```bash
   bazel run @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools:register_using_ioc_train_service -- \
     --address=localhost:17080 \
     --scene_object_id="ai.intrinsic.$WORKPIECE" \
     --pose_estimator_id="ai.intrinsic.${WORKPIECE}_estimator_text" \
     --refinement_iters=5 \
     --confidence_threshold=0.15 \
     --visibility_threshold=0.15 \
     --text_prompt="$TEXT_PROMPT"
   ```

4. Run one cycle with the antipodal mesh planner:

   ```bash
   bazel run //src:omts_app -- \
     --address=localhost:17080 \
     --config=configs/lab_bb_01/app_config.yaml \
     --workpiece="$WORKPIECE" \
     --grasp_planner=antipodal_mesh \
     --grasp_simulation_only
   ```

## Limits

- Only parallel-jaw grippers have a model, and only the Hand-E is configured.
- `antipodal_mesh` plans top-down grasps only.
- Mass, friction and holding force are not modeled. The simulated gripper
  welds the part to a finger when it closes while touching it, and holds
  nothing otherwise. A simulated lift shows that the fingers reached the
  part, not that the grasp would hold.
- In the Lab BB-01 simulation the table's collision surface is 22 mm below the
  visible table, so parts rest slightly sunk into it; and Gazebo pads mesh
  colliders when it decomposes them into convex pieces, so the fingers stop
  short of curved parts (about 2.6 mm per side on the banana).
