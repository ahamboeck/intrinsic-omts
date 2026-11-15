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

"""Vision and 3D camera hardware interfaces and Orbbec SBL implementation."""

import abc

from intrinsic.assets import id_utils
from intrinsic.perception.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments, provided

from src.core.config import VisionConfig
from src.utils.script_utils import create_dwell_task, create_python_script
from src.utils.workpiece_pose import update_detected_workpiece_pose


def _get_camera_resource(
  solution: deployments.Solution,
  camera_name: str,
) -> provided.ResourceHandle:
  """Resolves the camera resource handle from solution resources."""
  if isinstance(solution.resources, dict) and camera_name in solution.resources:
    return solution.resources[camera_name]
  try:
    return solution.resources[camera_name]
  except (KeyError, AttributeError, TypeError) as exc:
    raise ValueError(
      f"Camera resource '{camera_name}' not found in solution resources."
    ) from exc


def _get_perception_resource(
  solution: deployments.Solution,
  service_name: str,
) -> provided.ResourceHandle:
  """Resolves the perception service resource handle from solution resources."""
  if (
    isinstance(solution.resources, dict) and service_name in solution.resources
  ):
    return solution.resources[service_name]
  try:
    return solution.resources[service_name]
  except (KeyError, AttributeError, TypeError) as exc:
    raise ValueError(
      f"Perception service resource '{service_name}' not found in solution resources."
    ) from exc


class VisionInterface(abc.ABC):
  """Abstract interface for perception acquisition and pose estimation."""

  @abc.abstractmethod
  def build_capture_image_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to trigger camera image acquisition."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_perception_and_spawn_task(
    self,
    parent_object: str,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to capture RGB-D, estimate 6D poses, and update the workpiece pose."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Orbbec 3D camera perception adapter using Intrinsic Core SBL perception skills."""

  def __init__(
    self,
    solution: deployments.Solution,
    config: VisionConfig,
    log_debug_data: bool = True,
  ) -> None:
    """Initializes the Orbbec 3D vision adapter.

    Args:
        solution: Connected SBL deployment instance.
        config: Scoped VisionConfig defining camera, estimator, and sensor IDs.
        log_debug_data: Whether to log debug point clouds and images in SBL.
    """
    self._solution = solution
    self._camera_name = config.camera_name
    self._perception_service_name = config.perception_service_name
    self._pose_estimator_id = config.pose_estimator_id
    self._scene_object_id = config.scene_object_id
    self._sensor_ids = list(config.sensor_ids)
    self._min_num_instances = config.min_num_instances
    self._min_safe_z = config.min_safe_z
    self._log_debug_data = log_debug_data

    # Resolve resource handles strictly by configured name
    self._camera_resource = _get_camera_resource(solution, config.camera_name)
    self._perception_resource = _get_perception_resource(
      solution, config.perception_service_name
    )

  def build_capture_image_task(self, name: str | None = None) -> bt.Node:
    """Builds a single image capture task."""
    task_name = name or f"Capture Image ({self._camera_name})"
    action = self._solution.skills.ai.intrinsic.capture_images(
      camera=self._camera_resource,
      sensor_ids=self._sensor_ids,
      log_debug_data=self._log_debug_data,
    )
    return bt.Task(action=action, name=task_name)

  def build_perception_and_spawn_task(
    self,
    parent_object: str,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds the pipeline to capture RGB-D, estimate 6D poses, and update the workpiece pose.

    Grasp planning is left to the grasp planner, which runs on the pose this
    pipeline writes.

    Args:
        parent_object: World object the estimated pose is expressed in.
        max_tries: Attempts before the pipeline gives up.
        retry_delay_sec: Dwell between attempts.
        name: Optional custom behavior tree task name.

    Returns:
        Behavior tree node running the perception pipeline.
    """
    task_name = name or "Perception & Workpiece Pose Update Pipeline"

    skills = self._solution.skills

    # 1. Capture RGB-D Images
    capture_action = skills.ai.intrinsic.capture_images(
      camera=self._camera_resource,
      sensor_ids=self._sensor_ids,
      log_debug_data=self._log_debug_data,
    )
    capture_task = bt.Task(
      action=capture_action, name="1. Capture RGB-D Images"
    )

    # 2. Estimate 6D Poses via Multi-View / FoundationPose
    pkg = (
      id_utils.package_from(self._pose_estimator_id)
      if id_utils.is_id(self._pose_estimator_id)
      else "ai.intrinsic"
    )
    est_name = (
      id_utils.name_from(self._pose_estimator_id)
      if id_utils.is_id(self._pose_estimator_id)
      else self._pose_estimator_id
    )

    pose_estimator_proto = pose_estimator_id_pb2.PoseEstimatorId(
      id=est_name,
      package=pkg,
    )

    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
      perception=self._perception_resource,
      pose_estimator=pose_estimator_proto,
      capture_data=[capture_action.result.capture_data],
      min_num_instances=self._min_num_instances,
      log_debug_data=self._log_debug_data,
    )
    estimate_task = bt.Task(
      action=estimate_action, name="2. Estimate 6D Workpiece Poses"
    )

    # 3. Workpiece pose update via bt.PythonScript
    calc_task = self._create_pose_update_script_task(
      estimate_action=estimate_action,
      parent_object=parent_object,
    )

    acquisition_and_calc_seq = bt.Sequence(
      name="Perception Capture, Estimation & Frame Calculation",
      children=[
        capture_task,
        estimate_task,
        calc_task,
      ],
    )
    if max_tries > 1:
      recovery_task = create_dwell_task(
        dwell_time_sec=retry_delay_sec,
        solution=self._solution,
        task_name=f"Perception Retry Dwell ({retry_delay_sec}s)",
      )
      return bt.Retry(
        max_tries=max_tries,
        child=acquisition_and_calc_seq,
        recovery=recovery_task,
        name=task_name,
      )
    return acquisition_and_calc_seq

  def _create_pose_update_script_task(
    self,
    estimate_action: bt.ActionBase,
    parent_object: str,
  ) -> bt.Task:
    """Builds the bt.PythonScript task writing the first estimate into the world."""
    first_est = estimate_action.result.estimates[0].root_t_target
    calc_script = create_python_script(
      self._solution,
      update_detected_workpiece_pose,
      [
        ("float", "pos_x", first_est.position.x),
        ("float", "pos_y", first_est.position.y),
        ("float", "pos_z", first_est.position.z),
        ("float", "ori_x", first_est.orientation.x),
        ("float", "ori_y", first_est.orientation.y),
        ("float", "ori_z", first_est.orientation.z),
        ("float", "ori_w", first_est.orientation.w),
        ("string", "parent_object", parent_object),
        ("string", "camera_name", self._camera_name),
        ("string", "target_scene_object_id", self._scene_object_id),
        ("float", "min_safe_z", self._min_safe_z),
      ],
    )
    task_name = "3. Update Detected Workpiece Pose"
    return bt.Task(action=calc_script, name=task_name)
