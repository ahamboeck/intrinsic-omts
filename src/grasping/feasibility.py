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

"""Checks a planned grasp with Core's motion planner.

Planners that run as skills use this instead of their own IK and collision
checks, so every planner checks its grasps the same way and with Core's own
collision models of the gripper, the part and the cell.
"""

import grpc
import numpy as np
from absl import logging
from intrinsic.icon.proto import joint_space_pb2
from intrinsic.math.python import data_types, proto_conversion
from intrinsic.motion_planning import motion_planner_client
from intrinsic.motion_planning.proto import motion_target_pb2
from intrinsic.motion_planning.proto.v1 import (
  motion_specification_pb2,
  robot_specification_pb2,
)
from intrinsic.world.proto import collision_settings_pb2

from src.utils.math_utils import create_transform_node_ref


def grasp_is_feasible(
  motion_planner: motion_planner_client.MotionPlannerClient,
  robot_object: str,
  tool_object: str,
  tool_frame: str,
  grasp: np.ndarray,
  pregrasp: np.ndarray,
  collision_settings: collision_settings_pb2.CollisionSettings,
) -> bool:
  """Returns whether the robot can reach the pregrasp and move to the grasp.

  Solves collision-checked inverse kinematics at the pregrasp, then plans a
  collision-checked Cartesian-linear path from there to the grasp. Both are
  checked against the world as it is, so open the gripper first if the grasp
  is planned with open jaws.

  Args:
    motion_planner: Core motion planner client.
    robot_object: Object World name of the robot arm.
    tool_object: Object World object owning the tool frame.
    tool_frame: Moving tool frame placed at the grasp.
    grasp: 4x4 world pose of the tool frame at the grasp.
    pregrasp: 4x4 world pose of the tool frame at the pregrasp.
    collision_settings: Collision settings for both checks.

  Returns:
    True if both the IK and the linear path were found.
  """
  tool = create_transform_node_ref(tool_object, tool_frame)
  root = create_transform_node_ref("root")
  target = motion_target_pb2.CartesianMotionTarget(
    tool=tool,
    frame=root,
    offset=proto_conversion.pose_to_proto(
      data_types.Pose3.from_matrix4x4(pregrasp)
    ),
  )
  try:
    result = motion_planner.compute_ik(
      robot_object,
      target,
      options=motion_planner_client.IKOptions(
        max_num_solutions=1, collision_settings=collision_settings
      ),
    )
  except grpc.RpcError as error:
    logging.info("Pregrasp IK failed: %s %s", error.code(), error.details())
    return False
  if not result.solutions:
    return False
  robot = robot_specification_pb2.RobotSpecification(
    start_configuration=joint_space_pb2.JointVec(joints=result.solutions[0])
  )
  robot.robot_reference.object_id.by_name.object_name = robot_object
  segment = motion_specification_pb2.MotionSegment(
    motion_type=motion_specification_pb2.MotionSegment.LINEAR,
    collision_settings=collision_settings,
  )
  segment.target.cartesian_pose.moving_frame.CopyFrom(tool)
  segment.target.cartesian_pose.target_frame.CopyFrom(root)
  segment.target.cartesian_pose.target_frame_offset.CopyFrom(
    proto_conversion.pose_to_proto(data_types.Pose3.from_matrix4x4(grasp))
  )
  try:
    response = motion_planner.plan_path(
      robot,
      motion_specification_pb2.MotionSpecification(motion_segments=[segment]),
    )
  except grpc.RpcError as error:
    logging.info("Grasp approach failed: %s %s", error.code(), error.details())
    return False
  return bool(response.path.waypoints)
