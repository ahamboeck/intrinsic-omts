# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Frames and tasks for planners that re-plan the grasp at the unload.

With `GraspExecution.replan_at_unload`, the unload plans a new grasp into
separate frames, so the infeed frames stay available for the return. The pick
saves the part's pose relative to its grasp, and the return solves its tool
targets from that saved pose and the current grip. Planners never see these
frames; the behaviours derive them from the configured grasp frame names.
"""

from intrinsic.solutions import behavior_tree as bt

from src.core.config import AppConfig, FramesConfig
from src.hardware.robot import Robot
from src.utils import regrasp_frames
from src.utils.script_utils import create_python_script


def unload_frames(frames: FramesConfig) -> tuple[str, str]:
  """Returns the (grasp, pregrasp) frames the unload plans into."""
  return f"{frames.grasp_frame}_unload", f"{frames.pregrasp_frame}_unload"


def return_frames(frames: FramesConfig) -> tuple[str, str]:
  """Returns the (grasp, pregrasp) tool targets solved for the return."""
  return f"{frames.grasp_frame}_return", f"{frames.pregrasp_frame}_return"


def workpiece_frame(frames: FramesConfig) -> str:
  """Returns the frame holding the part's pose relative to the infeed grasp."""
  return f"{frames.grasp_frame}_workpiece"


def build_save_workpiece_pose_task(robot: Robot, config: AppConfig) -> bt.Task:
  """Builds the task saving the part's pose relative to the infeed grasp."""
  return bt.Task(
    name="Save Workpiece Pose at Grasp",
    action=create_python_script(
      getattr(robot, "_solution", None),
      regrasp_frames.save_workpiece_at_grasp,
      [
        ("string", "workpiece_object_name", config.cycle.workpiece_id),
        ("string", "grasp_frame_name", config.frames.grasp_frame),
        ("string", "workpiece_frame_name", workpiece_frame(config.frames)),
      ],
    ),
  )


def build_update_return_frames_task(robot: Robot, config: AppConfig) -> bt.Task:
  """Builds the task solving the return targets from the current grip."""
  grip_grasp, grip_pregrasp = unload_frames(config.frames)
  return_grasp, return_pregrasp = return_frames(config.frames)
  return bt.Task(
    name="Update Re-Grasp Return Frames",
    action=create_python_script(
      getattr(robot, "_solution", None),
      regrasp_frames.update_regrasp_return_frames,
      [
        ("string", "workpiece_object_name", config.cycle.workpiece_id),
        ("string", "tool_object_name", config.robot.tool_object_name),
        ("string", "tool_frame_name", config.robot.tool_frame_name),
        ("string", "workpiece_frame_name", workpiece_frame(config.frames)),
        ("string", "grip_grasp_frame_name", grip_grasp),
        ("string", "grip_pregrasp_frame_name", grip_pregrasp),
        ("string", "return_grasp_frame_name", return_grasp),
        ("string", "return_pregrasp_frame_name", return_pregrasp),
      ],
    ),
  )
