# BB-01 multi-view perception benchmark

Target: `//tools/pose_estimation:run_viewpoint_benchmark`.
This is a **simulation-only acquisition/evaluation workflow**, separate from
machine tending. It moves the arm between camera viewpoints, captures RGB/depth,
and calls the deployed pose-estimation service. It never grasps, commands the
gripper, updates workpiece belief, resets the simulator, or redeploys services.

## Repeatable comparison

1. Use the same BB-01 simulation scene, stationary object, camera calibration,
   CAD/estimator, exposure, starting arm configuration and simulation routing.
   Verify the arm hardware module is routed to the simulator before execution.
2. Generate **one** saved plan. Its center is the simulator object's world XYZ,
   not a detected pose. A horizontal circle uses **world Z**, not object-local Z
   (the object can be lying on its side). Camera optical +Z points at the center;
   optical -Y is approximately world up. The default radius, height above center,
   and initial azimuth come from the current RGB optical sensor position.
3. Run against original deployed services and corrected deployed services with
   that **same plan file**. Do not generate a new plan per version.
4. Keep different output directories and accurate deployment provenance. The
   collector does not decide which bugs are enabled or switch versions itself.

Simulator state is used for the benchmark's acquisition path and evaluation
only, never an inference ROI, pose prior, initial candidate or grasp input.
The native capture is saved before the normal Core request-preparation helper
runs. This helper is also used by the multi-view inference skill. The benchmark
calls the camera/service APIs directly to archive the exact inputs and to retain
empty results; it does not run the pick behavior or enforce min-num-instances.

## Save the plan (no motion)

Use absolute paths: Bazel may run the binary from its runfiles directory.

```sh
bazel run //tools/pose_estimation:run_viewpoint_benchmark -- plan \
  --address=192.168.10.3:17080 \
  --plan=/tmp/opencode/bb01_circle.json --views=12
```

This reads the simulated object, camera calibration and tool transform. It does
not capture an image or move the arm. Existing plans are never overwritten.
Override `--radius` and `--height` in metres if the starting view is unsuitable.
`--rgb-sensor=1` and `--depth-sensor=4` are sensor IDs, resolved to driver names.

**Targets are not preflighted.** A UR3e may not reach the entire circle with this
hand-eye mounting. Aim-at-center is not proof of visibility: inspect images for
gripper/enclosure occlusion. Start with a small number of views when first
validating the path. No collision exclusions are added. Moves use Core's normal
motion planning/collision checks; a failed move is recorded and **aborts** rather
than continuing from an unknown arm state. Moves are discrete planned hops,
not a guaranteed circular Cartesian trajectory. There is no automatic return
move or scene reset at the end.

## Collect each version (moves the simulated arm)

Prepare a JSON provenance file for **each deployed version**, for example:

```json
{
  "omts_revision": "<exact revision plus any deployed working-tree changes>",
  "core_version": "<published pin or exact modified revision>",
  "solution_artifact": "<installed solution artifact identity>",
  "foundationpose_artifact": "<installed model identity or hash>",
  "joint_ranking_fix": false,
  "depth_registration_fix": false
}
```

These are operator-provided claims, explicitly marked unverified in `run.json`.
Do not substitute the current checkout revision for what is actually deployed.
The stock baseline is OMTS upstream main with its published Core pin, not Core
main. Preserve/install its artifacts separately; this collector does not change
your branches, build settings or deployment. The same collector can target both
deployments, subject to camera/service API compatibility.

After verifying simulation routing and approving the camera path:

```sh
bazel run //tools/pose_estimation:run_viewpoint_benchmark -- run \
  --address=192.168.10.3:17080 \
  --plan=/tmp/opencode/bb01_circle.json \
  --cad=/tmp/opencode/omts_estimator_cad.json \
  --provenance=/tmp/opencode/baseline_deployment.json \
  --label=baseline --output=/tmp/opencode/bb01_baseline \
  --repeats=3 --execute-simulation
```

Then, after explicitly deploying the corrected version and restoring the same
scene/starting arm configuration, repeat with `--label=corrected`, its provenance
file, and `--output=/tmp/opencode/bb01_corrected`. Do not rerun `plan`.
Existing run directories are never overwritten. No deployment or movement has
been performed merely by adding this workflow.

`run` requires both `--execute-simulation` and a deployment reporting
`is_simulated=True`. It refuses a busy executive. Run it exclusively: another
client must not move the robot/change executive operations during collection.
The saved hand-eye calibration must agree within 0.1 mm/0.1 degrees. The object
must agree with the plan within 0.5 mm/0.1 degrees before each move. These are
benchmark consistency gates, not relaxed grasp or collision checks.

## Outputs

Each run contains:

- `run.json`: label, UTC start time, estimator/frame convention, CAD SHA-256,
  saved-plan digest, operator provenance and safety scope.
- `plan.json`: identical camera targets and original ground-truth center.
- `results.csv`: one row per detection, or an explicit no-detection/failure row.
  Includes view ID/repeat, timestamp, score, world-space actual/detected/camera
  translations in metres and 3x3 rotation matrices, position error in mm, raw
  rotation angle, unsigned per-axis angles and image path.
- `view_NNN_RRR/`: `rgb.png`, native `depth.npy`, original `capture.pb`, capture
  metadata/calibration/ground truth in `capture.json`, exact prepared
  `request.pb`, `response.pb`/`response.json`, final ground truth, and
  `comparison.png` (same RGB: green actual CAD on the left, magenta detections
  on the right). Rendering is wireframe projection, not an occlusion model.

No detection is chosen using ground truth. Every returned detection is retained.
The benchmark is designed for one known isolated workpiece; with multiple
objects, rows are not reliable object associations. Scores must not be compared
as calibrated confidence across original and corrected ranking procedures.

The default `--estimate-frame=camera` means service returns RGB optical-relative
poses; they are composed with the **measured capture sensor transform**. Select
`root` only for a service actually returning world-relative poses. No yaw/axis
heuristic or belief-pose substitution is applied.

`rotation_error_deg` is the full SO(3) error. The separate
`cuboid_symmetry_rotation_error_deg` allows the four 180-degree sign-flip
symmetries of a featureless rectangular cuboid; it does **not** allow swapping
the unequal CAD axes or assert that textured surfaces are symmetric.

Capture and simulator queries are not exposure-synchronized. Ground truth is
sampled before/after capture and after inference. `stable=False` flags motion
over 0.5 mm or 0.1 degrees; it does not establish absence of motion between
samples. `cad_in_image` checks CAD vertices against the RGB frustum only.
`valid_for_comparison` additionally requires stability and an achieved camera
pose within 2 mm/0.5 degrees of its planned target. Review occlusion manually,
and include failure/no-detection rates, not only successful-pose averages.
Camera settling defaults to one second, configurable with `--settle-seconds`.

## Side-by-side CSV

```sh
bazel run //tools/pose_estimation:run_viewpoint_benchmark -- compare \
  --baseline=/tmp/opencode/bb01_baseline/results.csv \
  --corrected=/tmp/opencode/bb01_corrected/results.csv \
  --output=/tmp/opencode/bb01_comparison.csv
```

The offline join requires identical plan digests and keys rows by view ID/repeat.
It prefixes columns `baseline_` and `corrected_`, retaining missing samples and
failures. Multiple rows for one sample are marked ambiguous rather than selecting
the lowest-error detection. Compare accuracy, visibility, achieved camera poses,
stability and failure rates together. Different captured pixels/noise between
live sweeps remain a confounder; archived requests enable later controlled
same-input replay, but replay is not implemented by this command.

Validation: geometry, projection, joins, simulation/busy guards, and mocked
collection/archiving are covered by unit tests. Reachability, optical calibration,
simulation routing, actual camera/service compatibility and real sweep visibility
still need simulation integration validation before claiming repeatability.
