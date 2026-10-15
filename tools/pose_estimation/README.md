# Pose Estimation Registration & Inference CLI (`tools/pose_estimation/`)

CLI utilities for registering CAD scene objects as FoundationPose estimators via
the Intrinsic Core Train Service (`IocTrainService`) and executing standalone 6D
pose estimation.

## CLI Targets

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/pose_estimation:register_using_train_service` | [`register_using_train_service.py`](register_using_train_service.py) | Generates and sideloads a FoundationPose estimator asset from an installed CAD scene object via `IocTrainService`. |
| `//tools/pose_estimation:run_pose_estimation` | [`run_pose_estimation.py`](run_pose_estimation.py) | Runs `capture_images` + `estimate_pose_multi_view` on `solution.executive` and prints 6D translations, quaternions, and RPY angles. |
| `//tools/pose_estimation:run_viewpoint_benchmark` | [`run_viewpoint_benchmark.py`](run_viewpoint_benchmark.py) | Simulation-only BB-01 camera-circle collection; saves images, ground truth, detections, and paired comparison CSV. |

See [BB-01 multi-view benchmark](../../docs/BB01_VIEWPOINT_BENCHMARK.md) for the
save-plan → original/corrected sweeps → side-by-side CSV workflow. Camera motion
is explicitly opt-in, and no deployment or grasp is performed by this tool.

## 1. Registering a Pose Estimator Data Asset

Creates and sideloads a pose estimator asset from an installed scene object:

```bash
bazel run //tools/pose_estimation:register_using_train_service -- \
  --address="localhost:17080" \
  --scene_object_id="ai.intrinsic.raw_stock_2x3x5" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --refinement_iters=3 \
  --confidence_threshold=0.6 \
  --visibility_threshold=0.6
```

* `--address`: gRPC address of the running solution deployment.
* `--scene_object_id`: Asset ID of the target CAD scene object (`<package>.<name>`).
* `--pose_estimator_id`: Asset ID to assign to the generated pose estimator (`<package>.<name>`).
* `--refinement_iters`: Number of FoundationPose refinement iterations (default: `3`).
* `--confidence_threshold`: Minimum detector confidence score threshold (default: `0.6`).
* `--visibility_threshold`: Minimum unoccluded visibility fraction threshold (default: `0.6`).

## 2. Running Pose Estimation

Executes RGB-D capture and multi-view 6D pose estimation using a registered pose
estimator asset:

```bash
bazel run //tools/pose_estimation:run_pose_estimation -- \
  --address="localhost:17080" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --camera_name="orbbec_camera" \
  --sensor_ids="1,4" \
  --service_name="pose_estimator_service" \
  --min_num_instances=1 \
  --log_debug_data=true
```

* `--address`: gRPC address of the running solution deployment.
* `--pose_estimator_id`: Asset ID of the registered pose estimator.
* `--camera_name`: Camera resource name (defaults to `orbbec_camera` or auto-detected `CameraConfig` resource).
* `--sensor_ids`: Comma-separated camera stream IDs (`1,4` for RGB + Depth on Orbbec Gemini 335Le).
* `--service_name`: Pose estimator service resource name (default: `pose_estimator_service`).
* `--min_num_instances`: Minimum required detected instances before skill failure (default: `1`).
* `--log_debug_data`: Whether to log debug point clouds and images during execution (default: `true`).
* `--timeout_sec`: Optional inference timeout in seconds.

### Fresh RViz comparison

Publication is enabled by default (`--nopublish_pose_overlay` disables it).
On this host, the README raw-stock command uses the previously verified exact
CAD at `/tmp/opencode/omts_estimator_cad.json` and the ROS Pixi environment at
`../../experiments/native_viewers/pixi.toml`. Other setups must supply
`--overlay_cad_json` and, if needed, `--overlay_ros_manifest`. Missing overlay
dependencies produce a warning without changing the perception results.

Each successful run exports **all returned detections**, then starts or reuses
a persistent ROS publisher. It updates `/omts/actual_pose_snapshot` (green) and
`/omts/perceived_pose_snapshot` (magenta), including CAD, axes, and timestamps.
RViz may remain open between runs. No physical motion or belief update is added.

`//src:omts_app` now uses the same snapshot publisher during cycles by default.
Each successful perception attempt records its capture's `world_t_camera`, all
received estimates, and an evaluation-only simulator pose before the pick moves
the object. A host observer refreshes the existing two ROS topics from that
typed result; it does not rerun perception or use truth for planning. These
markers remain pre-pick snapshots, not live tracking of an attached workpiece.
Actual truth is sampled after inference, not at exposure time.
The app accepts `--nopublish_pose_overlay`, `--overlay_cad_json`, and
`--overlay_ros_manifest`. Missing CAD disables this optional feature; unavailable
ROS or simulator data is diagnostic only and does not fail the cycle.

- `--overlay_estimate_frame=camera` is the default for the inspected deployed IOC
  service: despite the result field's `root_t_target` name, it returns
  camera-relative poses. Each is composed with **this capture's**
  `world_t_camera`. Use `root` only for a service returning world-relative poses.
- `--overlay_object_name=raw_stock_2x3x5` chooses the simulator comparison object.
  The actual pose is sampled after inference, not synchronized to camera exposure.
  Keep the object stationary for an accurate comparison. If simulator state is
  unavailable, only perceived markers are shown; there is no belief fallback.
- An empty successful result clears previous perceived detections. A failed run
  leaves the previous timestamped snapshot unchanged. This is not continuous
  perception or a guarantee that displayed poses are current.
- The host snapshot is `/tmp/opencode/omts_perception_overlay.json`; publisher
  diagnostics are in `/tmp/opencode/omts_perception_overlay.log`. One publisher
  owns the default topic pair; concurrent runs show the last completed export.

### Next steps

1. Confirm the active deployment was launched with `--operation_mode=sim`;
   `--simulation_mode=reality` alone does not confirm simulation-only operation.
2. Restore the simulator's initial state after the failed pick, then run one
   `omts_app` cycle with `cuboid_center` and your matching `$SETUP` configuration.
3. In RViz, verify both snapshot topics refresh and the log reports
   `Cycle pose overlay refreshed`. Leave the belief-mesh height offset unchanged.
4. Run two cycles and check both refreshes, flange attachment, and release.
   Scripted attachment is not proof of a physically valid grasp.
5. Retest baseline perception from `view`, then repeat matched simulation trials
   and archive results with source/runtime revisions. Lab BB-01 has no CNC;
   use the default OMTS setup for the full machine-tending comparison.
