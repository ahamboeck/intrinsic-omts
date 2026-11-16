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

"""Skill that turns the vacuum of an EOAT suction gripper on or off."""

import grpc
from absl import logging
from intrinsic.assets import interface_utils
from intrinsic.assets.dependencies import utils as asset_utils
from intrinsic.hardware.gripper.eoat import (
  eoat_service_pb2,
  eoat_service_pb2_grpc,
)
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface
from intrinsic.world.proto import object_world_refs_pb2

PARAMETERIZATION_ERROR = 10200
GRIPPER_SERVICE_ERROR = 10300


def _suction_gripper_interface_uri() -> str:
  service = eoat_service_pb2.DESCRIPTOR.services_by_name["SuctionGripper"]
  return f"{interface_utils.GRPC_URI_PREFIX}{service.full_name}"


class ControlSuctionGripper(skill_interface.Skill):
  """Sends grasp (vacuum on) or release (vacuum off) to a suction gripper."""

  def execute(self, request, context):
    del context
    params = request.params
    command = params.WhichOneof("command")
    if command not in ("grasp", "release"):
      raise skill_interface.SkillError(
        PARAMETERIZATION_ERROR, "Set either `grasp` or `release`."
      )
    stub = eoat_service_pb2_grpc.SuctionGripperStub(
      asset_utils.connect(
        params.suction_gripper, _suction_gripper_interface_uri()
      )
    )
    logging.info("Suction gripper: %s", command)
    try:
      if command == "grasp":
        stub.Grasp(params.grasp)
      else:
        stub.Release(params.release)
    except grpc.RpcError as error:
      details = getattr(error, "details", lambda: None)() or str(error)
      raise skill_interface.SkillError(
        GRIPPER_SERVICE_ERROR,
        f"Suction gripper service RPC failed: {details}",
      ) from error

  def get_footprint(self, request, context):
    del context
    return footprint_pb2.Footprint(
      object_reservation=[
        footprint_pb2.ObjectWorldReservation(
          type=footprint_pb2.ObjectWorldReservation.WRITE,
          object=object_world_refs_pb2.ObjectReferenceByName(
            object_name=request.params.suction_gripper.object.name
          ),
        )
      ],
    )
