# Experimental mesh backend: Lab BB-01 simulation

The `mesh` backend now plugs into the existing infeed pick subtree. Fresh camera
capture and localization update the belief object; the planner transforms the
actual object-coordinate CAD, runs contact-pair/approach/local collision checks,
and publishes the same `root/grasp` and `root/pre_grasp` frames. It does not read
Gazebo object state or use recorded poses. There is no old-grasper fallback.

Mesh uses a planned linear approach directly to the contact pose, followed by
the existing close/attach/linear-lift sequence. It skips force touchdown and the
pre-close retract because they would invalidate the selected mesh contacts.
The other backends retain their existing behavior.

## Opt-in configuration

```yaml
grasp:
  planner: mesh
  mesh_simulation_only: true
  mesh_cad_json: /absolute/path/to/installed-object-cad.json
  mesh_pregrasp_distance_m: 0.10
  mesh_max_approach_tilt_deg: 2.0
```

CAD JSON contains `vertices` and `triangles` arrays, already in metres and the
localized object's coordinate frame. Only exact duplicate seam vertices are
reindexed. Supply the installed estimator/deployed geometry, not a cuboid proxy.
The backend uses the modeled Hand-E TCP, finger geometry and 60 mm opening;
it is not a generic gripper adapter. Output frames currently must be on `root`.

CLI overrides are `--grasp_planner=mesh --mesh_cad_json=... --mesh_simulation_only`.
**The normal application runs the full machine-tending tree, not just a pick.**
For a pick-only experiment, build `build_pick_from_infeed_subtree` with this
planner and no machine adapter, as done for the live attempt below.

`mesh_simulation_only` is an acknowledgement, not automatic hardware detection.
Verify arm and gripper routes before execution. Executive `REALITY` mode means
execute against the connected deployment; it does not mean physical hardware.

## Limits and live result

This is local mesh selection, not full robot-feasible candidate selection.
Existing motion skills plan/check each motion against the robot and belief
scene. They do not preflight the entire approach/attached lift for every mesh
candidate or retry another candidate on planning failure. The existing pick
allows intended gripper/object contact during approach and retains its existing
post-attachment collision exclusions. Simulator sticky attachment is not proof
of a two-pad antipodal grasp. Do not use this experimental backend on hardware.

On 2026-10-11, the running simulation was tested with both perception services
unchanged. The arm used the Gazebo hardware stub; gripper commands were explicitly
routed to `/gripper/gripper_action_controller/gripper_cmd` on the Hand-E simulation
driver. The infeed pick moved to the view frame, captured/localized freshly and
executed the embedded mesh script successfully. Planning then rejected the pose:
**20 contact pairs, 0 approach proposals** within the 2-degree tilt limit.
The sequence failed before opening for the pick, grasp approach, attachment or
lift. No successful pick is claimed, and no geometry or tolerance was relaxed.

Operation: `244b76a43096386d00608094c0df9f17`.
Local diagnostics: `/tmp/opencode/omts_mesh_live_pick_operation.pb` and
`/tmp/opencode/omts_mesh_live_pick_result.json`. The perception bug fixes remain
undeployed; rejection alone does not establish the cause of the current pose.

A subsequent fresh-perception, **planning-only** rerun raised the runtime cap to
500 contact pairs and the sampling budget to 20,000, keeping all acceptance
checks unchanged. It returned **500 contact pairs, 0 approach proposals**;
no robot motion was commanded. More contact sampling did not resolve the
approach rejection. The runtime now retains those larger search budgets.
Operation: `fc6c2026064e10dc039234df52ccb5b2`.
Diagnostics: `/tmp/opencode/omts_mesh_search_500_result.json` and
`/tmp/opencode/omts_mesh_search_500_operation.pb`.

The next planning-only rerun added baseline width/tilt statistics and focused
sampling. Source faces now enter the sampling distribution only if their inward
normal (the ray's closing direction) admits the configured world-down approach.
Sampling remains area-uniform among those faces; topology, opposing normals,
opening clearance and all later local/motion checks are unchanged. Unfiltered
sampling is retained solely for diagnostic comparison, not as a grasp fallback.

Fresh perception produced 500 baseline pairs, all 50.800 mm wide, all requiring
83.282 degrees of approach tilt. Width rejected 0; the 2-degree approach limit
rejected all 500. The focused search returned 0 pairs and 0 proposals. This
identifies approach orientation rather than the pair cap or opening as the
sampled rejection cause; it does not establish global grasp impossibility.
The perceived object Z axis (its 50.8 mm dimension) is almost world-vertical.
No tolerance was relaxed and no robot motion was commanded.
Operation: `823d431fc15f9a42dd02075075a28259`.
Diagnostics: `/tmp/opencode/omts_mesh_focused_search_result.json` and
`/tmp/opencode/omts_mesh_focused_search_operation.pb`.

## Checks

```sh
bazel test --test_output=errors //tests/unit:test_mesh_runtime \
  //tests/unit:test_behaviors //tests/unit:test_config \
  //tests/unit:test_hardware_adapters
bazel build //src:omts_app
```
