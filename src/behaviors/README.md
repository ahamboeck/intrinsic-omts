# Behavior Trees & Motion Subtrees (`src/behaviors/`)

Composable SBL Behavior Tree builders and motion primitives orchestrating the
end-to-end machine tending cycle.

## Modules

| Module | Entrypoint Function | Role in Machine Tending Cycle |
| :--- | :--- | :--- |
| [`machine_tending_bt.py`](machine_tending_bt.py) | `build_machine_tending_behavior_tree` | Assembles all 5 subtrees into a single `bt.Sequence` and wraps it in `bt.Loop(max_times=...)` for multi-cycle (`> 1`) or continuous (`<= 0`) execution. |
| [`motions.py`](motions.py) | `create_move_to_frame_task`, `create_compliant_touchdown_task`, `create_relative_retract_task` | Reusable building blocks for Cartesian frame alignment, force-controlled contact (`move_to_contact`), and linear `-Z` tool retracts (`RelativePoseEquality`). |
| [`pick.py`](pick.py) | `build_pick_from_infeed_subtree` | **Subtree 1**: Optional CNC door/vise prep, view pose alignment, 3D vision pose estimation & workpiece pose update, grasp planning, compliant touchdown and 3 cm `-Z` retract or a direct approach, grasp, world attach, and linear lift. |
| [`load_machine.py`](load_machine.py) | `build_load_machine_subtree` | **Subtree 2**: Door/vise open verification, blended/direct approach to CNC entry & vise, compliant seating into jaws, vise clamp, release, world detach, and linear exit. |
| [`machining.py`](machining.py) | `build_machining_handshake_subtree` | **Subtree 3**: Robot standby outside enclosure, CNC door close, 0.5 s cycle-start DIO pulse, and cycle-complete wait/timeout. |
| [`unload_machine.py`](unload_machine.py) | `build_unload_machine_subtree` | **Subtree 4**: Door/vise open, machine & vise approach, compliant touchdown to machined part, 3 cm `-Z` retract, grasp, world attach, 3 cm `-Z` lift clear of vise jaws, and linear extraction. |
| [`regrasp.py`](regrasp.py) | `build_save_workpiece_pose_task`, `build_update_return_frames_task` | Frames and tasks for planners that re-plan at the unload: unload grasp frames, the part pose saved at the pick, and grip-corrected return targets. |
| [`return_infeed.py`](return_infeed.py) | `build_return_to_infeed_subtree` | **Subtree 5**: Blended/direct approach to infeed placement frame, compliant table touchdown, release, world detach, linear retract, and return to `view` frame. |

## Key Orchestration Rules

* **Single Executive Submission**: All subtrees are composed into one
  `bt.BehaviorTree` executed via a single `solution.executive.run(tree)` call.
* **Optional CNC Hardware (`machine=None`)**: When `config.machine` is `None`
  (e.g., `lab_bb_01`), door, vise, and cycle handshake nodes are automatically
  omitted without branching at runtime.
* **Segment-Scoped Collision Rules**: Contact and close-proximity segments attach
  targeted `CollisionRule` exclusions (e.g., excluding `[gripper, workpiece]`
  during `-Z` linear retracts or `[gripper, vise]` during vise approach) rather
  than disabling collision checking globally.
* **Grasp Planner and `GraspExecution`**: `main.py` always passes the planner
  selected in the `grasp` config section. The pick runs it right after the
  perception pipeline, so it plans on the pose perception has just
  established, and the subtrees read its `GraspExecution` to decide when the
  gripper opens, whether the tool touches down or moves directly onto the
  grasp, and whether the unload re-plans and the return corrects for the new
  grip ([`regrasp.py`](regrasp.py)). Without a planner (`grasp_planner=None`),
  the existing grasp frames are used with a touchdown. See
  [`src/grasping/README.md`](../grasping/README.md).
