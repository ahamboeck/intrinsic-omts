# Pure mesh grasping — contacts, top-down poses and local checks

This package provides **numeric contracts, bounded geometric contact search,
world-top-down pose proposals and conservative local gripper/object checks**.
`propose_contact_pairs()` returns contact-pair proposals, not complete gripper
poses. `propose_top_down_grasps()` adds poses; `check_local_grasp()` separately
checks their declared local geometry. No backend is registered and existing picks
are unchanged. There is no physical/hardware calibration or Gazebo success claim.
See the [implementation plan](../../docs/MESH_GRASPING_PLAN.md).

## Contracts

- All lengths are **metres**. Mesh vertices and contact points/normals use the
  **object-local frame**, not a world frame or a gripper TCP frame.
- `TriangleMesh` holds indexed triangle geometry; `ParallelJawLimits` contains
  aperture limits only, not palm/finger collision geometry or TCP calibration.
- `ContactPairOptions.opening_clearance_m` is **total extra opening**, not an
  amount per finger. Search requires
  `min_contact_separation_m <= pair.separation_m` and
  `pair.separation_m + opening_clearance_m <= max_opening_m`.
- `SurfaceContact` retains its source triangle index. Normal length must be
  within `UNIT_NORMAL_TOLERANCE = 1e-6` of one (absolute length tolerance, no
  relative tolerance). Normals are not silently normalized.
- `ContactPair` requires distinct points and derives positive finite
  `separation_m`; pair order does not assign physical left/right jaw joints.
  Construction alone does not verify antipodal alignment or aperture limits.

Dataclasses are frozen and **tuple storage is required at every geometry level**;
lists are rejected, not silently converted. Scalars accept built-in integers
and floats (not booleans); index/budget/seed fields require built-in integers.
Malformed input raises `ValueError`. Nonempty geometry, finite coordinates,
in-range indices and nondegenerate faces are checked. Degeneracy is evaluated at
floating-point precision without an arbitrary minimum-area threshold; edge
lengths must remain finite and representable.

The dataclass checks do **not** certify a closed shell, outward winding,
self-intersection freedom or contact surface membership. A single triangle is
a valid structural fixture, not a supported search object. Nor do these types
prove collision clearance, approach/closure feasibility, robot reachability,
frictional stability or an executable grasp. Search adds the guarantees and
limitations below.

## Search behavior and supported meshes

The first search supports **one closed, consistently outward-wound, embedded
two-manifold triangle shell**. Faces must share vertex indices at seams; an
unwelded STL triangle soup is not supported. It checks duplicate faces, two
oppositely directed uses of every edge, connected faces, connected vertex fans
and positive signed volume. It does not repair meshes or fall back to a box.

**Self-intersection detection is not implemented.** Absence of self-intersections
is an input precondition, not a certified result of the topology checks. Scans,
assemblies, nested/disconnected shells and inward-wound surfaces are unsupported.
Malformed or detected unsupported geometry raises `ValueError`.

1. Sample triangle interiors proportional to triangle area, using a private
   seeded RNG (no global random-state changes).
2. Cast along the sampled triangle's inward normal and take the **first exit**,
   not a farther surface across an empty notch. Skip ambiguous edge/vertex hits.
3. Require each outward normal to align with its outward closing-line direction
   within `options.max_normal_angle_rad` (default **5 degrees**, in radians).
   The allowed range is `0 <= angle < pi/2`; this is a geometric tolerance, not
   a friction coefficient or a force-closure/stability guarantee.
4. Enforce jaw limits and **total** extra opening clearance. Suppress pairs with
   matching endpoints in either order within `1e-6` of the mesh diagonal.
5. Stop after `max_pairs` accepted pairs or `max_samples` attempted surface
   samples (default **2000**), whichever happens first. Output order follows
   accepted sampling order, not a quality score. There is no guarantee of filling
   the output budget or finding every possible antipodal closing line.

Ray/area calculations use translated coordinates normalized by the mesh's
bounding-box diagonal. Geometric ambiguity uses `1e-10` in normalized length and
barycentric coordinates, and a relative `1e-10` parallel/skinny-face threshold.
Normalized enclosed volume must exceed `1e-10`. Dot-product alignment comparisons
allow `1e-12` of roundoff; aperture comparisons use the returned separation with
no extra opening tolerance. Features below this resolution may be rejected or
missed. The search is bounded, with ray cost proportional to
`max_samples * triangle_count`; no spatial index or new dependencies are added.

Identical inputs and seed produce identical results within the same runtime.
`[]` means a bounded search found no accepted pairs, **not** that no grasp exists.
Contact pairs alone lack approach/TCP/collision guarantees. The next layers below
add explicit geometric proposals and checks, not robot/scene or physical validity.

## Top-down pose contract

`RigidTransform` stores **rotation basis columns**, not rows or quaternions,
and a translation in metres. Construction requires an orthonormal,
right-handed rotation within `1e-9` absolute dot-product tolerance.

`propose_top_down_grasps(tuple(pairs), profile, world_T_object=..., ...)`:

- Requires a fresh numeric object pose. Derives object-local approach from
  **world `(0, 0, -1)`**, including when the object is tilted. There is no
  side/angled approach generation in the first PR.
- Rejects pairs whose closing line is not perpendicular to world down (within
  `1e-9` dot-product roundoff allowance); it never tilts the approach to fit.
- Places the center at the contact midpoint, with `+X` closing, `+Z` approach,
  `+Y = +Z cross +X`. Returns two yaw choices, 180 degrees apart, per compatible
  pair, so asymmetric gripper geometry can be checked separately. These are
  internal alternatives, **not** an OMTS retry list.
- Computes `object_T_tcp = object_T_center * center_T_tcp`, including arbitrary
  nonidentity numeric TCP rotation/translation. Pregrasp uses the same TCP
  rotation and translates opposite approach by positive `pregrasp_distance_m`.
- Requires explicit `open_width_m`: the geometric aperture corresponding to
  the intended open command, within profile limits and strictly larger than
  contact separation. It does not silently choose a maximum opening.

Outputs are immutable `GraspPoseProposal` values. They are not locally checked
until passed through the separate checker; neither layer runs IK or ranks poses.

## Reused Hand-E model and calibration limits

`ParallelJawProfile` describes end-effector-only envelopes at maximum opening,
fixed palm geometry, symmetric prismatic jaw motion along +/-X, one declared
inward contact-pad face per jaw, and a center-to-TCP transform. Other compatible
profiles can replace it without changing search. This is **not** an arbitrary
multi-finger/rotary-jaw model.

`hande_collision_profile()` is a versioned numeric snapshot derived from:

- [Core's Hand-E SDF](https://github.com/intrinsic-ai/intrinsic-core/blob/7695b55a60d0b0623e1db0f3cd5cda10053f965f/intrinsic_control/intrinsic/models/assets/standard_tooling/robotiq_hande/robotiq_hande/robotiq_hande.sdf)
  (same file in release `20260922.0`). SHA256:
  `d418de6468022ba5604a6862bc71f124c38c8e64623c27cff5c3f1952b82e60d`.
- [OMTS's finger-offset patch](../../bazel/patches/robotiq_hande_finger_offset.patch),
  adding +5/-5 mm to finger1/finger2 link X. SHA256:
  `3dd7f43245e6d38674762f46fcb87b288f386d762a736fbc4bccd37bb2b71da4`.

All **11 SDF collision primitives** are enclosed: five body/IO shapes and three
shapes on each jaw, including racks. Boxes are retained; spheres/cylinders use
oriented enclosing boxes. An offline analytic support-bound comparison against
the patched SDF verified all centers and primitive containment on 2026-10-05.
The snapshot itself imports no asset parser or Core client.

The patched simplified fingertip spheres have radius 3 mm and centers at
X=+/-33 mm, Z=150 mm at zero joint displacement. Their inward apexes define a
**model gap** of 60 mm; 25 mm inward travel per jaw reduces it to 10 mm. Grasp
center is `(0,0,150 mm)` in body coordinates; Core `tool_frame` is
`(0,0,128 mm)`, giving `center_T_tcp.translation=(0,0,-22 mm)`.
These numbers describe the modeled sphere contact, **not measured flat pads**.
Enclosing the simplified colliders does not certify containment of visual CAD
or physical hardware. Verify/replace the pad geometry and calibration before
execution; MoveIt's 140 mm tool offset is not interchangeable with Core's 128 mm.

**Command mapping remains unresolved:** SDF axes make increasing finger joint
position move inward, but current OMTS configs label `open_position=0.025` and
`close_position=0.0`. The adapter must establish the actual command-to-joint/gap
mapping and contact-stop/overtravel behavior. The smoke example's 60 mm aperture
is a numeric modeled state, not authorization to send an OMTS open command.

## Conservative local validation

`check_local_grasp(mesh, proposal, profile)` rechecks supported mesh
topology/winding, triangle provenance and actual surface membership/normals,
aperture, center-to-contact geometry, TCP composition and pregrasp consistency.
Malformed/contradictory inputs raise `ValueError`. Valid but locally incompatible
geometry returns `LocalCheckResult(accepted=False, reason=..., stage=..., collider=...)`.

- Checks the **entire straight open approach** and **entire symmetric closure
  to first pad contact** using conservative swept oriented boxes. This includes
  intermediate collisions even when both endpoints are clear; it is not
  discretized pose sampling.
- Clips actual object triangles against each swept envelope and checks solid-angle
  containment, so a collider wholly inside the object is also rejected.
- Rejects palm/back/rack contact and pad side contact. Closure may touch only the
  designated inward pad face with a matching outward object normal, not penetrate
  it. Contact allowance is `1e-10` metres for floating-point roundoff; normal
  dot-product allowance is `1e-9`. All other touching/ambiguous hits reject.
  The modeled pad-face requirement is stricter than the search's default 5-degree
  normal tolerance: some contact pairs therefore cannot pass this gripper profile.

Conservative envelopes can cause false rejections. The supported mesh's absence
of self-intersections is still a precondition. Success means **passed declared
local checks only**. It excludes command overtravel/contact compliance, friction,
mass/CoM, arm/table/fixture collisions, attached lift, Core motion planning and
physical stability. Those are later gates, not optional safety checks.

The pure package uses only the Python standard library: no Core, NumPy, ROS,
MoveIt, simulator or asset-loading imports. Bazel targets follow OMTS's existing
Core-provided build macros; that is not a runtime SDK dependency.

## Verify step 1

From the OMTS repository root, run the focused contract suite:

```sh
python3 -B -m unittest discover -s tests/unit -p test_grasping_types.py -v
```

This needs no external Python packages, Core connection or hardware. All tests
must pass.
It covers valid construction, nested immutability, malformed geometry,
nonfinite values, index and aperture boundaries, normal tolerance, options and
derived contact separation.

The repository build-system check is:

```sh
bazel test --test_output=errors //tests/unit:test_grasping_types
```

Bazel requires the configured external repositories/toolchains.

## Verify step 2

```sh
python3 -B -m unittest discover -s tests/unit -p test_contact_pairs.py -v
bazel test --test_output=errors //tests/unit:test_grasping_types //tests/unit:test_contact_pairs
```

The search suite checks analytic box, concave L and U surfaces, real triangle
provenance, closing-line alignment, no contacts spanning empty notches, aperture
and clearance boundaries, area-weighted sampling, seeded repeatability, rigid
transforms, duplicate suppression, bounded output and unsupported topology.
Passing these tests is evidence for contact search, not a Gazebo pick.

## Ordered checklist for the current implementation

Run these from the **OMTS repository root**, in order. No cluster or hardware is
needed; the Python suites use only the standard library.

1. Contracts (22 tests):
   `python3 -B -m unittest discover -s tests/unit -p test_grasping_types.py -v`
2. Search (17 tests):
   `python3 -B -m unittest discover -s tests/unit -p test_contact_pairs.py -v`
3. Top-down frames/calibration (9 tests):
   `python3 -B -m unittest discover -s tests/unit -p test_grasp_poses.py -v`
4. Model/local sweeps (17 tests):
   `python3 -B -m unittest discover -s tests/unit -p test_local_grasp_checks.py -v`
5. Bazel packaging/toolchain:

   ```sh
   bazel test --test_output=errors \
     //tests/unit:test_grasping_types //tests/unit:test_contact_pairs \
     //tests/unit:test_grasp_poses //tests/unit:test_local_grasp_checks
   ```

6. Inspect a concave L fixture end to end, still without running OMTS:

   ```sh
   python3 -B - <<'PY'
   import runpy
   from src.grasping.contact_pairs import propose_contact_pairs
   from src.grasping.geometry import RigidTransform
   from src.grasping.hande_profile import hande_collision_profile
   from src.grasping.local_checks import check_local_grasp
   from src.grasping.poses import propose_top_down_grasps
   from src.grasping.types import ContactPairOptions

   mesh = runpy.run_path('tests/unit/test_contact_pairs.py')['l_profile']()
   profile = hande_collision_profile()
   pairs = propose_contact_pairs(mesh, profile.limits,
       options=ContactPairOptions(seed=42, max_pairs=20, opening_clearance_m=.001))
   poses = propose_top_down_grasps(tuple(pairs), profile,
       world_T_object=RigidTransform(), pregrasp_distance_m=.1, open_width_m=.06)
   results = [check_local_grasp(mesh, pose, profile) for pose in poses]
   print('contact pairs:', len(pairs))
   print('top-down proposals:', len(poses))
   print('passed declared local checks:', sum(r.accepted for r in results))
   for result in results:
       if not result.accepted:
           print(result.stage, result.collider, result.reason)
   PY
   ```

   The 2026-10-05 checkpoint produced **20 pairs, 26 top-down proposals, 14 local
   passes**. Counts are a reproducible checkpoint, not a public completeness
   guarantee. The tests require deterministic results and both accepted/rejected
   alternatives, not a specific random sample sequence.

7. **Not available yet:** geometry acquisition from Core, CAD/hardware calibration,
   command mapping, ranking/diversity, downstream collision-aware IK/motion,
   frame publication and Gazebo measured lift. Do not test by enabling a mesh
   backend or running a physical pick; no such backend was added.
