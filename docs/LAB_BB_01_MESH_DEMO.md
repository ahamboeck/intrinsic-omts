# Lab BB-01 mesh grasp demo (no motion)

This demo exports candidate TCP/pregrasp frames and continuous local collision
results from actual CAD and a **recorded perception pose**. It never connects to
the executive, publishes world frames, opens/closes the gripper, or moves the arm.
It is not yet a pick/lift demonstration: robot/scene feasibility is NOT checked.

## Inputs

CAD JSON: `vertices` in object coordinates/metres and `triangles` as index triples.
Use the installed estimator/deployed object CAD, not a differently transformed
visual. Exact duplicate export seams are reindexed; geometry is not simplified.

Pose JSON: `translation_m` and `rotation_basis_columns` (three orthonormal basis
**columns**, not matrix rows). Compose world_T_camera with camera_T_target from
perception. Do not provide simulator ground truth as the planning pose. Recorded
input is not fresh enough for execution; it is only for this offline demo.

```sh
bazel run //tools/grasping:mesh_demo -- \
  --cad=/tmp/opencode/omts_estimator_cad.json \
  --world-object-pose=/tmp/opencode/omts_demo_world_object_pose.json \
  > /tmp/opencode/omts_lab_bb_01_mesh_demo.json
```

The diagnostic input files are machine-local, not checked-in assets. The demo
defaults to 60 mm open width, 100 mm pregrasp distance and a 2-degree approach
tilt limit, using the modeled Hand-E profile. `--max-tilt-deg=0` exercises strict
top-down. Locally accepted candidates are **not** ranked robot-feasible grasps;
the report always says `execution_ready: false`.

## Before a live pick/lift demo

1. Deploy both perception fixes and capture a fresh, validated object pose.
2. Fetch the installed CAD and compose all mesh/object/camera transforms.
3. Add a bounded Core checker for pregrasp, contact-aware approach and attached
   lift in an isolated planning world; endpoint IK is insufficient.
4. Verify live TCP/profile calibration and aperture mapping. Publish frames only
   for a fully accepted candidate; do not reuse stale grasp frames on failure.
5. Review the no-motion plan, then explicitly authorize simulated pick/lift.
   The sticky Gazebo attachment is not proof of physical antipodal grasp success.

See `OMTS_MESH_GRASP_INTEGRATION_PLAN.md` for the remaining runtime gates.
