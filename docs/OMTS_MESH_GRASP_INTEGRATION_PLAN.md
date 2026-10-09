# OMTS mesh grasp integration: current cube pick and lift

**Agreed scope:** 2026-10-09. **Status:** implementation plan; integration and
simulation execution have not been validated.

## 1. Goal and decisions

Use the current cube and its **FoundationPose-estimated pose** to execute an
opt-in pick/lift with OMTS's pure Python mesh-grasp backend. Intrinsic continues
to own robot/scene motion planning and execution; Gazebo remains the simulator.

The user confirmed:

- First milestone is cube pick, lift and hold, not the full machine-tending cycle.
- Use the current FoundationPose pipeline from the first end-to-end test. Do not
  introduce a ground-truth-pose demo or make one a prerequisite.
- Preserve the current gripper and attachment behavior. Do not switch to a
  friction-only or a new attachment-assisted implementation for this feature.
- Simulation only; no physical robot activation.

“Our backend” means `src/grasping`, not the separate MoveIt experiment. Keep
`cuboid_center` as the default and preserve the existing MoveIt option. No new
segmentation model, MuJoCo backend, or broad behavior-tree rewrite is in scope.

## 2. Existing implementation and remaining gaps

The broader workspace feature list is stale for this checkout: contracts,
contact search, world-top-down pose proposals and scoped local collision checks
already exist. See [the geometry plan](MESH_GRASPING_PLAN.md) and
[`src/grasping/README.md`](../src/grasping/README.md) for exact guarantees.

Available today:

- Immutable metre-based mesh/contact/profile/pose contracts.
- Bounded seeded contact search on supported closed triangle shells.
- World-top-down TCP/grasp/pregrasp proposals with two yaw alternatives.
- Conservative continuous gripper/object approach and closure checks.
- An OMTS planner interface, factory, and post-perception planning seam.

Still missing:

- Acquisition of the cube's actual supported mesh and fresh world pose.
- Deployed gripper geometry, TCP and command-to-aperture verification.
- Candidate ranking/diversity and downstream robot/scene feasibility checks.
- A packaged runtime adapter, backend selection and frame publication.
- A generated-contact-compatible pick path and executed simulation evidence.

These geometric checks do not certify frictional stability, robot reachability,
scene clearance or physical pad calibration.

## 3. Current grasp semantics: preserve, inspect, label accurately

`src/behaviors/pick.py` currently closes the gripper, calls
`Robot.build_attach_object_task`, and then retracts.
`src/hardware/robot.py` builds the Intrinsic `attach_object_to_robot` skill.
This establishes an explicit application attachment step, **not proof of a
friction-only Gazebo grasp**. Whether the deployed simulator adds a constraint
or relies on contact/friction must be checked against the actual skill/runtime
and simulator configuration.

Keep that sequence's existing attachment semantics. Record the observed mechanism
in each run: a constrained/attachment-assisted lift must not be reported as proof
of frictional grasp stability. A friction-only benchmark is not part of this plan.

## 4. Implementation slices and acceptance gates

### P0 — freeze and verify the active cube/runtime baseline

1. Identify the active cube's asset/object name, dimensions, mesh units, mesh-to-
   entity transform and scene placement. Checked-in OMTS config names
   `raw_stock_2x3x5`; do not assume it is the deployed cube.
2. Record OMTS/Core revisions or archives, effective Hand-E patch, deployed image
   digests, simulation config and current backend. Use the actual OMTS dependency,
   not APIs seen only in the adjacent Core checkout.
3. Inspect the existing attachment skill and Gazebo configuration to establish
   current grasp semantics without changing them.
4. Capture a current FoundationPose result with target identity and available
   validity/freshness evidence. Diagnose invalid/no-estimate results explicitly;
   do not accept success-shaped fallback poses or use ground truth as a fallback.
   An identity transform alone is not proof of an invalid estimate.
5. Verify simulated Hand-E opening/closing motion, pad geometry, collision-model
   containment, and TCP mapping. The current profile uses Core's 128 mm tool
   offset, not MoveIt's 140 mm offset. SDF inward joint motion appears inconsistent
   with OMTS's `open_position=0.025` / `close_position=0.0` labels. Establish the
   deployed mapping and closure/contact behavior; do not silently swap commands.

**Gate:** a recorded, reproducible simulation configuration with known cube,
usable FoundationPose input, and a consistent gripper model/command mapping.
Any necessary correction to shared gripper behavior is a separately reviewed,
tested change, not an incidental backend patch.

### P1 — cube geometry bridge and bounded candidate selection

1. Acquire geometry through supported APIs/assets for the selected Core runtime.
   Normalize actual vertices into object-local metres and preserve entity/asset
   transforms. A cuboid primitive may be triangulated exactly; do not substitute
   a bounding box for a non-box surface or silently repair unsupported topology.
2. Read the fresh object pose produced by the current perception run, after
   localization. Keep geometry, object identity and pose provenance associated.
3. Run contact search, top-down pose construction and local sweep checks with
   explicit seed, sample/candidate budgets, clearance, aperture and pregrasp
   distance. Cache immutable geometry only if useful; never cache a stale pose.
4. Add simple documented deterministic ranking and pose deduplication/diversity.
   Scores are geometric preferences, not success probabilities.
5. Verify and use Intrinsic's actual downstream feasibility mechanism for
   pregrasp, contact approach and attached lift. Bound candidate work/deadlines;
   try the next candidate when one fails. Do not invent an IK API or select the
   first local pass without downstream checks.

**Gate:** transformed cube fixtures, unsupported/too-wide inputs, local rejection,
determinism, bounded work, and first-candidate-rejected/second-candidate-accepted
tests. No motion commands in this slice.

### P2 — plan-only OMTS adapter and runtime packaging

1. Implement `src/hardware/mesh_antipodal_planner.py` through the existing
   `GraspPlannerInterface`; extend factory, enum, validated config and Bazel
   dependencies with opt-in `mesh_antipodal` selection.
2. Package the pure geometry imports for remote execution. Source embedding via
   `load_python_script` does not bundle sibling modules. Prefer an existing
   supported packaged execution mechanism; a new persistent service needs a
   demonstrated lifecycle/dependency requirement.
3. Consume localization, compute/check alternatives, then publish exactly one
   grasp/pregrasp pair on success in the correct parent frame. Ensure failures
   cannot leave a partial update executable; use supported transactional or
   explicit validity/gating mechanisms rather than assuming multi-frame atomicity.
4. Invalid/stale/wrong-target pose, unsupported mesh, timeout, unavailable planner
   or no feasible candidate must fail the current run before pick motion. Never
   reuse previous frames or silently fall back to cuboid-center.
5. Provide a plan-only entry point that runs current perception and planning and
   reports poses/rejections without issuing arm/gripper commands. If capturing
   an input requires a camera-view move, do that separately and explicitly; do
   not hide it inside the no-motion plan-only operation.

**Gate:** factory/default/MoveIt regressions, config validation, remote package
smoke test, correct transform publication, stale/partial-frame failure tests,
and an actual FoundationPose-fed plan-only run with no actuator commands.

### P3 — narrow mesh-specific pick/lift path

1. Add an opt-in mesh execution path without changing cuboid or MoveIt behavior.
   Reach the generated contact pose before closing: open -> pregrasp -> linear
   contact approach -> close -> existing attachment step -> linear lift -> hold.
2. Do not use the existing seated approach's retract-before-close sequence for
   generated contacts. Do not add arbitrary touchdown/offset compensation to make
   the generated poses fit that sequence.
3. Keep scene collision checking enabled. Scope unavoidable intended-contact
   exclusions narrowly and document their limits; local gripper/object checks
   do not justify disabling arm/table/fixture collision checking.
4. Use the current FoundationPose pipeline, with the same validity/freshness
   gates as plan-only. Invalid perception stops the run rather than triggering a
   ground-truth or old-frame fallback.
5. Keep deployment explicitly in simulation mode. Stop on failures; do not
   automatically retry motions or switch backends. Define a safe stopped-run
   reset/release procedure for repeat trials.

**Gate:** behavior-tree ordering tests prove close precedes lift and existing
attachment is retained; cuboid and MoveIt ordering remains unchanged. Executed
Gazebo cube pick/lift uses a FoundationPose pose and the mesh-selected frames.

### P4 — measured demo and regression evidence

Use the workspace roadmap's proposed demo thresholds as the initial gate:

- At least **5 cm measured object lift** and **2 s hold**.
- Relative object-to-gripper drift below **5 mm / 5 degrees** during the hold.
- Report every attempted trial and failure; do not infer reliability from one
  successful pick or from an attachment constraint keeping drift small.

Measure simulator state for evaluation only; ground truth must not feed grasp
planning. Record starting height, object/gripper trajectories, perception result,
selected candidate, rejection reasons and the actual attachment mechanism.
Verify the executed contact/lift matches the selected plan, not merely that an
attached object follows the robot.

**Gate:** documented FoundationPose-fed pick/lift evidence, fault cases, existing
backend regressions, and an explicit distinction between integration success
and frictional stability. The milestone does not claim full CNC-cycle support.

## 5. Verification and delivery

- Run the four existing pure geometry suites first; do not treat their historical
  results as newly executed checks.
- Add focused geometry-bridge, candidate-selection, adapter/config/packaging and
  pick-order tests as each slice lands. Use fake runtime responses for failure
  paths; transform fixtures are unit tests, not a ground-truth simulation demo.
- Validate malformed mesh, no valid pose, wrong target, stale pose, no local pass,
  no downstream-feasible candidate, timeout and partial publication. None may
  proceed with old grasp frames.
- Run relevant Bazel targets, repository formatting/lint and affected behavior
  regressions before simulation. Record dependency/toolchain blockers precisely.
- Run packaged plan-only before explicitly authorized simulated motion. Broaden
  validation to existing cuboid and MoveIt paths according to changed shared code.
- Log revisions/image/model/scene/calibration hashes, backend choices, seed and
  budgets, perception validity, motion-policy flags, outcome and failure reason.
- Deliver cohesive reviewable changes: baseline evidence, geometry/selection,
  adapter/packaging, opt-in execution, then demo evidence. Do not commit or push
  automatically; the user owns those actions.

## 6. Remaining engineering discoveries, not deferred scope decisions

The implementer should investigate cube identity, public geometry access, runtime
packaging, downstream planning APIs, pose-validity/freshness signals and gripper/
attachment semantics. Ask the user again only if evidence requires a material
scope change, new service/dependency, Core/runtime upgrade, or modification of
current shared grasp behavior. Do not quietly change the agreed FoundationPose,
simulation-only, current-semantics scope.
