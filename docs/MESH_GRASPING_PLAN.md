# Mesh grasping: first implementation layout

**Status: steps 1–3 and scoped local checks implemented, not an enabled feature.**
Pure contracts, contact search, world-top-down pose construction and conservative
SDF-derived gripper/object sweep checks are available in `src/grasping`. See its
[README](../src/grasping/README.md) for exact guarantees, model/calibration limits
and an ordered runnable test checklist. No backend or frame publisher is registered.

## Goal and ownership

Add mesh-aware parallel-jaw grasping to OMTS incrementally, while keeping the
geometry algorithm reusable and existing cuboid behavior unchanged.

```text
Mesh + parallel-jaw limits
        -> contact-pair proposals                  [steps 1–2]
        -> calibrated pose proposals               [step 3]
        -> local modeled approach/closure checks    [implemented, scoped]
        -> CAD/calibration/command verification      [still required]
        -> OMTS adapter + Core scene/robot planning  [later]
        -> existing execution + Gazebo measurement  [later]
```

The pure package owns object-local geometry. It does not load assets, connect
to Core, run IK, command a gripper, mutate the world or select a CNC task policy.
OMTS owns those application decisions; Core retains robot/scene motion planning
and execution. Keep the separate MoveIt experiment out of this feature slice.

## Proposed package layout

Create only the files marked **first step** initially. Future files show the
intended boundaries, not a request to scaffold empty abstractions now.

| Path | Responsibility | Stage |
|---|---|---|
| `src/grasping/__init__.py` | Small package marker; no eager runtime imports | First step |
| `src/grasping/types.py` | Immutable mesh, jaw limits and contact-pair contracts | First step |
| `src/grasping/contact_pairs.py` | Pure search entry point; explicit placeholder initially | First step |
| `src/grasping/README.md` | Units, frames, output guarantees and limitations | First step |
| `src/grasping/BUILD` | Narrow library targets | First step |
| `tests/unit/test_grasping_types.py` | Contract and unfinished-entry-point tests | First step |
| `tests/unit/BUILD` | Add focused `test_grasping_types` target | First step |
| `tests/unit/test_contact_pairs.py` | Search behavior and geometric fixtures | Step 2 |
| `src/grasping/geometry.py` | Numeric column-basis rigid transforms | Step 3 |
| `src/grasping/profile.py` | Numeric symmetric parallel-jaw envelopes and TCP mapping | Step 3 |
| `src/grasping/hande_profile.py` | Patched Hand-E SDF collision-envelope snapshot | Step 3 |
| `src/grasping/poses.py` | Contact pairs to world-top-down pose proposals | Step 3 |
| `tests/unit/test_grasp_poses.py` | Frame, calibration and equivariance tests | Step 3 |
| `src/grasping/local_checks.py` | Conservative palm/finger, swept approach and closure checks | Scoped local gate |
| `tests/unit/test_local_grasp_checks.py` | Model contracts, continuous-sweep regressions and pipeline fixture | Scoped local gate |
| `src/hardware/mesh_antipodal_planner.py` | Existing OMTS planner interface implementation | Later |

Keep related numeric types together rather than creating a file per dataclass.
Do not introduce a service, dependency-injection framework, parser registry or
new application-level planner interface for the first algorithm.

## Step 1 — Implement interfaces and contract tests

### Public entry point

```python
def propose_contact_pairs(
  mesh: TriangleMesh,
  gripper: ParallelJawLimits,
  *,
  options: ContactPairOptions,
) -> list[ContactPair]:
  raise NotImplementedError("Contact-pair search is not implemented yet.")
```

This placeholder is intentional and must not be called from an application
backend. Returning `[]` here would incorrectly claim that a search ran and found
nothing. Do not invent box contacts or use the old prototype as a silent fallback.

### Input/output types

Use standard-library frozen dataclasses and immutable nested tuples. No NumPy,
Core protobuf, SDK pose type, ROS message or OMTS configuration object is needed.

| Type | Fields | Meaning |
|---|---|---|
| `TriangleMesh` | `vertices: tuple[Vec3, ...]`, `triangles: tuple[Triangle, ...]` | Indexed triangle surface already normalized into object-local metres |
| `ParallelJawLimits` | `min_contact_separation_m`, `max_opening_m` | Contact separation lower limit and actual maximum jaw opening |
| `ContactPairOptions` | `seed`, `max_pairs`, `opening_clearance_m` | Local randomness, output budget and total extra opening |
| `SurfaceContact` | `point: Vec3`, `outward_normal: Vec3`, `triangle_index: int` | Surface location, normal and triangle provenance in the input mesh |
| `ContactPair` | `first: SurfaceContact`, `second: SurfaceContact` | One candidate closing line, not a complete grasp |

`Vec3` is `tuple[float, float, float]`; `Triangle` is
`tuple[int, int, int]`. Field names involving lengths explicitly use metres.
For points, the mesh contract supplies the units and frame.

Return **a list of pairs**, even if the first consumer only inspects one. This
preserves alternatives when later pose or motion checks reject the first pair.
Two points alone do not establish antipodal alignment; normals are required.
Two points and normals still do not specify approach, roll, TCP or accessibility.

Start with `ParallelJawLimits`, not a supposedly complete `GripperModel` that
contains only two numbers. The full profile will later include palm/finger
geometry, jaw kinematics and calibration. The first hardware baseline is the
existing Hand-E, but its numeric dimensions must be verified before fixtures
are described as calibrated hardware.

### Validation and semantics

- Require nonempty vertices and triangles, three finite coordinates per vertex,
  three integer in-range indices per triangle, and nondegenerate triangles.
- Require finite jaw limits with
  `0 <= min_contact_separation_m <= max_opening_m`; require nonnegative finite
  clearance, an integer seed and a positive integer output budget.
- Define clearance as **total extra opening**, not clearance per finger.
  Once search exists, every pair must satisfy positive separation and
  `min_contact_separation_m <= separation`, plus
  `separation + opening_clearance_m <= max_opening_m`.
- Require finite contact points and unit normals and nonnegative triangle indices.
  A returned contact's triangle index must also be valid for its source mesh.
  Unit-normal tolerance belongs in a named, documented numeric check.
- Derive separation from the two points; do not store a second width value that
  can disagree. Pair ordering does not assign physical left/right jaw joints.
- Malformed values raise `ValueError`. The unimplemented entry point raises
  `NotImplementedError`. After step 2, `[]` means a valid bounded search found no
  pairs; it is not proof that no physical grasp exists.
- Structural type validation is **not** closed-shell, outward-winding or
  self-intersection certification. Step 2 must establish/document supported mesh
  topology before generating normals and contacts.
- A standalone contact constructor cannot establish surface membership or outward
  orientation. Those are generator guarantees tested against the input mesh.

Do not add scoring, friction coefficients, sampling internals, TCP transforms,
collision flags or opaque metadata until their contracts have a concrete use.

### First-step tests and acceptance

Write failing tests before implementing the types. Cover:

1. Valid mesh, jaw limits, options and contact construction.
2. Nested immutability, not just `frozen=True` with mutable list fields.
3. Nonfinite coordinates/limits, bad indices and degenerate triangle rejection.
4. Invalid aperture ranges, clearance and output-budget rejection.
5. Invalid contact normal and negative provenance index rejection.
6. Explicit `NotImplementedError`, never a fake no-result answer.
7. Pure imports without Core, ROS or MoveIt runtime modules.

Use existing Bazel `py_library`/`py_test` rules. The pure tests use standard-library
`unittest` to remain runnable without external Python packages. Build macros may come from Core; pure library dependencies must not
include its runtime clients. Follow `pyproject.toml` and the existing Ruff and
buildifier pre-commit configuration; Python uses two-space indentation.

First meaningful check from the OMTS repository:

```bash
rtk bazel test --test_output=errors //tests/unit:test_grasping_types
```

Also format/check changed files and inspect the final dependency graph/diff.
If dependency fetching blocks Bazel, record the exact blocker and distinguish any
direct Python test results from a successful Bazel run. No cluster is needed.

**Step 1 is complete when:** the focused contracts are implemented and tested,
their limitations are documented, and no factory/config/pick/perception behavior
has changed. A stub plus comments alone is not completion. It produces no grasp
and makes no simulator or robot success claim.

## Step 2 — Implement the first real contact-pair search

Implemented in `src/grasping/contact_pairs.py`, with box/L/U contract tests in
`tests/unit/test_contact_pairs.py`. Options now add `max_samples=2000` and
`max_normal_angle_rad=math.radians(5)`. See the package README for the supported
topology, numerical resolution, first-exit ray method and verification commands.
The step-1 placeholder described above is historical and has been replaced.

This step implements only `propose_contact_pairs`, keeping asset
loading, poses and hardware execution outside it.

1. Add an outward-wound closed box and synthetic concave L-profile as fixtures.
   These are simple controllable geometry, not certified Hand-E pick scenarios.
2. Validate supported closed-shell/winding assumptions; explicitly reject
   unsupported scans/assemblies. Document any self-intersection limitation.
3. Use a bounded seeded surface sampling/pairing method. Select sample/search
   budgets and normal-alignment tolerance with explicit semantics in this step.
4. Compute contact normals from actual triangles. Check alignment against the
   closing line, not merely that the two normals oppose each other.
5. Apply jaw limits/clearance, return bounded deterministic alternatives and
   handle a valid search with no results explicitly.

Tests must establish surface membership/provenance, normal/closing-line alignment,
opening bounds, identical-input/seed repeatability, bounded output and no-result
semantics. The L fixture must not invent contacts on a bounding-box face in empty
space. Test behavior, not the identity of the random samples.

**Step 2 is complete when:** real contact proposals pass those tests on supported
fixtures. They are still not collision-free, physically stable or executable.
Any angular/friction-cone assumption must be stated; do not advertise full force
closure, payload support or a success probability.

## Step 3 — Construct world-top-down full-pose proposals

Implemented without touching application runtime. The first PR is explicitly
**world-top-down only**; arbitrary side/angled approaches are deferred.

1. Define `RigidTransform` with numeric basis columns and `ParallelJawProfile`
   with modeled jaw limits, articulated collision envelopes and center-to-TCP
   mapping. Hardware/CAD calibration remains an explicit later verification gate.
2. Construct a right-handed grasp-center frame: `+X` closing,
   `+Z` pregrasp-to-grasp approach, `+Y = +Z × +X`.
3. Derive object-local approach from world down using fresh numeric
   `world_T_object`. Reject nonperpendicular closing lines rather than tilting the
   approach. Return two 180-degree yaw alternatives per compatible pair.
4. Apply the fixed center-to-TCP transform. Keep grasp-center and TCP conventions
   distinct; derive pregrasp from approach direction, not a world-Z offset.
5. Return immutable `GraspPoseProposal` with contact provenance, center and TCP
   grasp/pregrasp frames, and explicit geometric open width. Command-to-aperture
   mapping must be verified by the future adapter, not guessed here.

Tests: proper orthonormal rotations, nonidentity TCP calibration, positive approach
distance, correct pregrasp offset and rigid-transform equivariance. Specify
quaternion ordering/unit norm if the numeric representation uses quaternions.

**Step 3 is complete when:** these geometric pose proposals are tested. Do not call
them locally valid grasps before palm/finger, closure and approach checks exist.
Ranking/diversity and simulator success also remain separate acceptance gates.

The effective Hand-E profile must account for OMTS's finger-offset patch. Core
and MoveIt currently use different body-to-tool model offsets (128 vs 140 mm);
resolve conventions/calibration before cross-backend comparisons, not with an
unexplained compensating offset. No arm model belongs in the pure profile.

## Scoped local validation — implemented, not deployment certification

Reuse the effective patched Hand-E SDF's 11 primitive colliders as numeric
enclosing boxes, with symmetric jaw translations. Check actual mesh triangles
and containment against conservative **continuous** approach and closure sweeps,
including collisions between clear endpoints. No object bounding-box fallback,
discrete trajectory sampling or blanket gripper/object exemptions.

The checker validates provenance, normals and proposal/profile consistency.
Only inward contact-pad tangency is allowed at closure; palm/back/rack and other
pad contacts reject. Closure ends at first modeled pad contact, not after an
unverified command overtravel. Rejections include stage/collider diagnostics.

Tests cover simple box tangency, contained colliders, intermediate approach and
closure collisions, noncontact fingers, invalid provenance/poses, slanted contact
normals, coordinate changes, arbitrary TCP calibration and a deterministic
concave-L search-to-local-check pipeline. Passing is evidence for the declared
SDF-based envelopes, not physical pad/CAD calibration or an executable grasp.

Before integration, verify simplified-envelope containment against visual CAD,
real pad geometry, the 128 mm Core TCP convention, and the command mapping:
current OMTS `open_position=0.025`/`close_position=0.0` naming is opposite to the
inward motion indicated by the inspected SDF joint axes. Do not silently resolve
this by swapping commands or declaring a hardware aperture.

## Later integration: use the existing OMTS seam

The current application already provides these boundaries:

- [`src/hardware/grasping.py`](../src/hardware/grasping.py):
  `GraspPlannerInterface.build_plan_grasp_task(...)` updates preexisting
  grasp/pregrasp frames for an already-localized object at BT execution time.
- [`src/hardware/grasp_planners.py`](../src/hardware/grasp_planners.py):
  `create_grasp_planner` is the single backend factory.
- [`src/core/types.py`](../src/core/types.py) and
  [`src/core/config.py`](../src/core/config.py): backend enum and validated config.
- [`src/behaviors/pick.py`](../src/behaviors/pick.py): the optional planner task runs
  after perception and before opening/moving. Current planners require perception
  infeed; they do not localize the object themselves.

After verifying model/CAD/calibration/command consistency, add
ranking/deduplication/diversity. Only then implement the adapter and register
`mesh_antipodal`, preserving `cuboid_center` as default. The adapter reads fresh
world/entity geometry and transforms, converts to the pure contract and selects
a downstream-feasible alternative. It publishes only **one** grasp/pregrasp frame
pair on success, preserving the existing pick seam. Current pick execution does
not accept a candidate list or automatically retry failed motions.

Before wiring the BT, verify runtime packaging. OMTS submits a tree to Core;
[`load_python_script`](../src/utils/script_utils.py) embeds a module's source but
does not bundle its imported sibling modules. A local `src.grasping` import is
not evidence that it works inside remote `bt.PythonScript`. Prefer existing
mechanisms; do not introduce a distributed service without demonstrated need.
Use typed protobufs if a skill/service execution boundary is actually introduced.

Before any Gazebo pick, address the current seated approach's retract-before-close
semantics with a small opt-in path. Generated contacts must be reached before
closure. Broad task refactoring and physical robot activation remain out of scope.

## Dependency and contribution setup are separate decisions

At the 2026-10-04 source checkpoint, local OMTS is `8b3887c…`, local Core is
`7695b55…`, and [`MODULE.bazel`](../MODULE.bazel) still pins Core `20260922.0`.
The newest published Core release inspected is also `20260922.0`. A sibling
checkout update does not alter the Bazel dependency. This plan does not authorize
a fictional release bump or silently change the source/runtime compatibility pair.

If Core main is required, choose and verify an explicit development override or
reproducible source bundle, preserving materialized LFS assets and the effective
gripper patch. This is not necessary for the pure contract implementation.

Development is on `feature/mesh-contact-pair-contracts`, with `origin` pointing
to the user's fork and `upstream` to Intrinsic. Step 1 landed as `883685c`.
Follow-up work is organized into separate contact-search, top-down pose/profile,
local-check and documentation commits. Nothing was pushed and no PR was
published. GitHub CLI authentication remains unavailable;
SSH access to the fork works. Preserve preexisting edits, including the local
`.gitignore`. Before an upstream submission, confirm the
[contribution/CLA requirements](../CONTRIBUTING.md).

## Questions/defaults to confirm later

1. Accept a list of pairs with points, normals and triangle provenance?
   **Recommended: yes**, to keep downstream alternatives and testable contacts.
2. Use Hand-E as the first full gripper profile, with synthetic box/L tests first?
   **Recommended: yes**, without guessing hardware dimensions.
3. Keep the first implementation step strictly contracts/validation/tests?
   **Recommended: yes**; algorithm in step 2, pose construction in step 3.
4. For Core main, prefer a local development override or a reproducible main-commit
   bundle? Resolve separately before changing the dependency pin.
