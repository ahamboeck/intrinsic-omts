# Mesh grasp integration: status and next steps

**Updated:** 2026-10-20. Short working plan for `integration/mesh-grasp`. The
full scope, slices (P0–P4) and gates are in
[the integration plan](OMTS_MESH_GRASP_INTEGRATION_PLAN.md); this file only
records where the work stands and what to do next.

## What this branch contains

Based on upstream `main` (`7e9edf6`), linear history:

1. `fix(foundationpose): score pose candidates jointly`: same change as branch
   `fix-foundationpose` (open upstream PR).
2. `fix(foundationpose): treat invalid depth as missing`: same change as branch
   `fix-invalid-depth`.
3. The mesh grasp commits from `feature/mesh-grasp-demo` (contracts, contact
   search, top-down poses, Hand-E profile, sweeps, selection, plan-only demo).
   The older copy of the scoring fix (`7f393a8`) was left out.

When the two fixes are merged upstream, rebase onto `main` and drop commits 1–2.

Not included yet: the runtime integration (planner adapter, pick path, config,
runtime frames) and the perception evaluation tools. Both exist only in the WIP
commit `85363e2` / working tree of `intrinsic-omts-modified`, mixed with local
Core-upgrade build changes that upstream `main` already replaced.

## Perception status (simulation, `lab_bb_01`)

Errors against simulator truth, rotation adjusted for the cuboid's 180° symmetry:

| Camera pose | `main` | Both fixes |
|---|---|---|
| Lab detection pose (`view` frame) | 3.28 mm / 8.32° | 1.18 mm / 0.44° |
| Taught pose 1 | 3.00 mm / 6.10° | 1.24 mm / 0.26° |
| Taught pose 2 | 0.69 mm / 1.40° | 1.16 mm / 0.48° |
| Taught pose 3 | identity fallback | 1.12 mm / 0.70° |

"Both fixes" was measured with the scoring fix plus Core-side depth cleanup,
which in simulation has the same effect as the OMTS depth fix. This branch's
exact combination has not been run in simulation yet.

Known perception issue still open: in taught poses 2 and 3 a second detection
lands on the gripper (245–283 mm off) and scores **higher** than the part.
At the lab detection pose only the part was detected.

## Why the last mesh run found no grasp

On 2026-10-11 (old perception, see `LAB_BB_01_MESH_RUNTIME.md` in
`intrinsic-omts-modified`), planning found 500 contact pairs, all across the
50.8 mm side, and 0 approaches: the perceived 50.8 mm axis was almost vertical,
needing 83° tilt against a 2° limit.

In the current reset scene, simulator truth has the **76.2 mm side vertical**
and the 50.8 mm side horizontal, which a top-down grasp with the 60 mm Hand-E
opening should reach. The old result was therefore likely caused by perception
(6–8° errors at that time) or a different scene state. Not yet verified.

## Next steps

1. **Port the runtime integration** from `85363e2` as its own commit(s): only
   the mesh path (`src/hardware/mesh_grasp_planner.py`,
   `src/grasping/runtime_frames.py`, the related changes in `pick.py`,
   `config.py`, `types.py`, `robot.py`, `vision.py`, `grasp_planners.py`,
   `grasping.py`, `main.py`, BUILD targets, `test_mesh_runtime.py` and the
   affected behavior/adapter tests, `LAB_BB_01_MESH_RUNTIME.md`). Leave out
   Core-upgrade build changes and benchmark tools. Gate: all unit tests pass;
   default `cuboid_center` and MoveIt behavior unchanged.
2. **Deploy this branch in simulation.** The `omts-sim` launcher only runs the
   frozen A–D sources, so this needs a new frozen source/version for this
   branch (Core unchanged; no Core rebuild).
3. **Planning-only run at the lab detection pose, no motion.** Record contact
   pairs, approach proposals and tilt statistics; compare with the 0 proposals
   above. Check, for evaluation only, that the perceived 50.8 mm axis is
   horizontal like simulator truth.
4. **Branch on the result.**
   - Proposals found: continue with P1 ranking and downstream feasibility,
     then P3 (mesh pick path) and P4 (measured lift/hold evidence).
   - Still 0 proposals: check mesh frame vs. estimator frame and axis
     conventions before changing any limits. Do not relax tolerances.

## Open items to keep in mind

- **Gripper false positive** (above): the highest-scoring detection can be the
  gripper. Any "take the best detection" logic would pick it.
- **Gripper state during detection:** the lab cycle closes the gripper before
  perception (`Close Gripper (Clear View)`); the benchmark collector does not.
  Test whether that removes the false positive.
- **Hand-E command mapping** (P0 item 5) is still unverified.
- **GPU memory** of joint scoring is not measured (expected ~1.9x for the
  scoring step with `batch_size: 128`).
- Simulation only; no hardware motion.
