# Actual versus perceived CAD snapshot in RViz

`tools/grasping/pose_overlay.py` publishes the same actual CAD twice in `root`:
green at the diagnostic simulator pose and magenta at the composed perception
pose. Each has RGB X/Y/Z arrows and a timestamped **SNAPSHOT** label. Object Z is
the 50.8 mm short axis for the inspected Lab BB-01 CAD. This is an offline
snapshot overlay, not continuous perception, TF, or a robot-control node.

## Automatic refresh after the perception CLI

The README's `bazel run //tools/pose_estimation:run_pose_estimation` command now
exports fresh results and starts/reuses the native ROS publisher automatically.
Keep `pixi run omts-rviz` open: both displays refresh after a successful run.
Perceived markers use every received detection (no belief-pose substitution),
composed with that capture's `world_t_camera` by default. Actual markers use a
read-only simulator pose sampled after inference. Objects should be stationary
because capture and simulator evaluation are not synchronized.

See [perception CLI options](../tools/pose_estimation/README.md#fresh-rviz-comparison)
for CAD/environment configuration and service-frame selection. The verified
raw-stock CAD and native ROS Pixi workspace are required on this host. Overlay
failure is reported without altering inference results. The persistent process
watches `/tmp/opencode/omts_perception_overlay.json`; it does not call perception
itself. Failed runs retain the previous timestamped snapshot.

## Manual diagnostic snapshot

The diagnostic supplies world-coordinate `fresh_world_object` and
`actual_after_evaluation_only` poses. Rotations are basis **columns**. CAD
vertices must be in object coordinates and metres. World-space triangle/arrow
points use identity marker poses, so transforms are applied exactly once.
No scale fitting, centering, axis snapping, or bounding-box substitute is used.

Run from the OMTS repository root with ROS 2 `rclpy`, `geometry_msgs` and
`visualization_msgs` available:

```sh
python -B -m tools.grasping.pose_overlay \
  --cad /tmp/opencode/omts_estimator_cad.json \
  --diagnostic /tmp/opencode/omts_axis_diagnostic_result.json
rviz2 -d tools/grasping/pose_overlay.rviz
```

The supplied RViz view is focused on the Lab BB-01 object position. Separate
MarkerArray topics: `/omts/actual_pose_snapshot` (green) and
`/omts/perceived_pose_snapshot` (magenta); fixed frame: `root`. If adding to an
existing RViz window, add one display per topic with Reliable / Transient Local
QoS. Each topic includes its own CAD, axes, and label. Override names with
`--actual-topic` and `--perceived-topic`. Markers are republished at 1 Hz, but
their input data does not refresh; recapture/export and restart for new poses.

The host's existing `pixi run omts-rviz` configuration now also includes the
separate **Actual SNAPSHOT (green)** and **Perceived SNAPSHOT (magenta)** displays. The normal workcell display
is unchanged. The overlay publisher must still be running; launching RViz does
not start it automatically; the perception CLI does. Toggle each pose independently in
the Displays panel. Hide the workcell's `raw_stock_2x3x5/base_link` namespace if
its normal object marker obscures the comparison.

On this host the existing ROS environment is:

```sh
pixi run --manifest-path ../../experiments/native_viewers/pixi.toml \
  --frozen -e ros python -B -m tools.grasping.pose_overlay \
  --cad /tmp/opencode/omts_estimator_cad.json \
  --diagnostic /tmp/opencode/omts_axis_diagnostic_result.json
pixi run --manifest-path ../../experiments/native_viewers/pixi.toml \
  --frozen -e ros env -u LD_PRELOAD rviz2 -d tools/grasping/pose_overlay.rviz
```

That environment uses ROS 2 Lyrical / `rmw_zenoh_cpp`, domain 0, the local Zenoh
router, and X11/XWayland. The ROS dependencies intentionally remain outside the
OMTS Bazel SDK environment. Pure geometry has Bazel regression coverage:
`bazel test //tests/unit:test_pose_overlay`.

The initial snapshot is from 2026-10-11 22:40 UTC. Expected discrepancy: actual
short Z axis horizontal, perceived short Z axis almost vertical. Both mesh
origins are retained, including the measured 11.3 mm position error.
