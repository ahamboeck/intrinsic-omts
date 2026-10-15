"""Cycle diagnostics use captured estimates, never updated belief or late truth."""

import types
import unittest
from unittest import mock

from google.protobuf import (
  any_pb2,
  descriptor_pb2,
  descriptor_pool,
  message_factory,
)
from intrinsic.math.proto import pose_pb2
from intrinsic.perception.proto.v1 import capture_data_pb2
from intrinsic.perception.proto.v1 import pose_estimate_in_root_pb2
from intrinsic.solutions import proto_building as pb

from tools.pose_estimation.cycle_overlay import (
  CyclePoseOverlay,
  record_cycle_snapshot,
)


def result_type():
  file = descriptor_pb2.FileDescriptorProto(
    name="cycle_overlay_test.proto",
    package="cycle_overlay_test",
    syntax="proto3",
    dependency=[
      capture_data_pb2.DESCRIPTOR.name,
      pose_estimate_in_root_pb2.DESCRIPTOR.name,
      pose_pb2.DESCRIPTOR.name,
    ],
  )
  message = file.message_type.add(name="Result")
  for number, name, typename in (
    (1, "capture", ".intrinsic_proto.perception.v1.CaptureData"),
    (2, "estimates", ".intrinsic_proto.perception.v1.PoseEstimateInRoot"),
    (3, "actual", ".intrinsic_proto.Pose"),
  ):
    message.field.add(
      name=name,
      number=number,
      type=11,
      type_name=typename,
      label=3 if name == "estimates" else 1,
    )
  for number, name in ((4, "finished_at_utc"), (5, "warning")):
    message.field.add(name=name, number=number, type=9, label=1)
  descriptor = descriptor_pool.Default().AddSerializedFile(
    file.SerializeToString()
  )
  return message_factory.GetMessageClass(
    descriptor.message_types_by_name["Result"]
  )


class CycleOverlayTest(unittest.TestCase):
  def setUp(self):
    self.Result = result_type()
    self.capture = capture_data_pb2.CaptureData()
    self.capture.world_t_camera.orientation.w = 1
    self.capture.world_t_camera.position.x = 1
    self.estimate = pose_estimate_in_root_pb2.PoseEstimateInRoot(
      id="part", score=0.7
    )
    self.estimate.root_t_target.orientation.w = 1
    self.estimate.root_t_target.position.x = 0.2
    self.params = types.SimpleNamespace(
      capture=self.capture, estimates=[self.estimate], object_name="stock"
    )
    self.context = mock.MagicMock()
    actual = pose_pb2.Pose()
    actual.orientation.w = 1
    actual.position.x = 1.25
    self.context.object_world.stub.GetTransform.return_value = (
      types.SimpleNamespace(a_t_b=actual)
    )
    self.overlay = CyclePoseOverlay(
      mock.MagicMock(), "stock", "estimator", "cad.json", "/workspace"
    )
    self.overlay.results = [types.SimpleNamespace(value_type=self.Result)]

  def packed(self, result):
    packed = any_pb2.Any()
    packed.Pack(result)
    return packed

  def test_task_serializes_typed_result_and_binds_current_capture_and_estimates(
    self,
  ):
    files = {}

    def collect(descriptor):
      for dependency in descriptor.dependencies:
        collect(dependency)
      files[descriptor.name] = descriptor_pb2.FileDescriptorProto.FromString(
        descriptor.serialized_pb
      )

    collect(self.Result.DESCRIPTOR.file)
    signature = pb.SignatureWithArgs(
      signature=pb.Signature(
        file_descriptor_set=descriptor_pb2.FileDescriptorSet(
          file=list(files.values())
        ),
        parameter_message_full_name="",
        return_value_message_full_name=self.Result.DESCRIPTOR.full_name,
      ),
      params_message=None,
      blackboard_params={},
    )
    builder = self.overlay.solution.proto_builder.create_signature_with_args
    builder.return_value = signature
    capture, estimate = mock.MagicMock(), mock.MagicMock()
    task = self.overlay.build_task(capture, estimate)
    parameters = builder.call_args.kwargs["parameters"].fields
    self.assertIs(parameters[0].arg, capture.result.capture_data)
    self.assertIs(parameters[1].arg, estimate.result.estimates)
    self.assertTrue(parameters[1].repeated)
    self.assertEqual(parameters[2].arg, "stock")
    self.assertEqual(
      self.overlay.results[-1].value_type.DESCRIPTOR.full_name,
      self.Result.DESCRIPTOR.full_name,
    )
    serialized = task.proto
    self.assertEqual(len(serialized.fallback.tries), 2)
    self.assertEqual(
      serialized.fallback.tries[
        0
      ].node.task.execute_code.return_value_message_full_name,
      self.Result.DESCRIPTOR.full_name,
    )

  def test_records_all_received_poses_and_bounded_simulator_read_before_motion(
    self,
  ):
    self.params.estimates.append(self.estimate)
    result = record_cycle_snapshot(self.context, self.params, self.Result)
    self.assertEqual(len(result.estimates), 2)
    self.assertEqual(result.estimates[0], self.estimate)
    self.assertEqual(result.capture, self.capture)
    request = self.context.object_world.stub.GetTransform.call_args.args[0]
    self.assertEqual(request.world_id, "sim_world")
    self.assertEqual(request.node_b.by_name.object.object_name, "stock")
    self.assertEqual(
      self.context.object_world.stub.GetTransform.call_args.kwargs,
      {"timeout": 1.0},
    )
    self.context.object_world.update_transform.assert_not_called()

  def test_publishes_fresh_cycles_once_using_saved_not_late_actual_pose(self):
    board = mock.MagicMock()
    first = record_cycle_snapshot(self.context, self.params, self.Result)
    first.finished_at_utc = "cycle-1"
    second = record_cycle_snapshot(self.context, self.params, self.Result)
    second.finished_at_utc = "cycle-2"
    second.estimates[0].root_t_target.position.x = 0.4
    board.get_value_any.side_effect = [
      self.packed(r) for r in (first, first, second)
    ]
    with mock.patch(
      "tools.pose_estimation.cycle_overlay.publish_snapshot"
    ) as publish:
      for _ in range(3):
        self.overlay.publish_new(board)
    self.assertEqual(publish.call_count, 2)
    snapshots = [call.args[0] for call in publish.call_args_list]
    self.assertEqual(
      [s["finished_at_utc"] for s in snapshots], ["cycle-1", "cycle-2"]
    )
    self.assertAlmostEqual(
      snapshots[0]["perceived_objects"][0]["pose"]["translation_m"][0], 1.2
    )
    self.assertAlmostEqual(
      snapshots[1]["perceived_objects"][0]["pose"]["translation_m"][0], 1.4
    )
    self.assertEqual(
      snapshots[1]["actual_after_evaluation_only"]["translation_m"][0], 1.25
    )

  def test_missing_simulator_does_not_substitute_belief_or_fail_perception(
    self,
  ):
    self.context.object_world.stub.GetTransform.side_effect = ValueError(
      "no sim world"
    )
    result = record_cycle_snapshot(self.context, self.params, self.Result)
    self.assertFalse(result.HasField("actual"))
    board = mock.MagicMock()
    board.get_value_any.return_value = self.packed(result)
    with mock.patch(
      "tools.pose_estimation.cycle_overlay.publish_snapshot"
    ) as publish:
      self.overlay.publish_new(board)
    self.assertIsNone(publish.call_args.args[0]["actual_after_evaluation_only"])

  def test_missing_capture_transform_and_publisher_failure_are_optional(self):
    self.capture.ClearField("world_t_camera")
    result = record_cycle_snapshot(self.context, self.params, self.Result)
    board = mock.MagicMock()
    board.get_value_any.return_value = self.packed(result)
    with mock.patch(
      "tools.pose_estimation.cycle_overlay.publish_snapshot"
    ) as publish:
      self.overlay.publish_new(board)
      publish.assert_not_called()
    self.context.object_world.stub.GetTransform.assert_not_called()
    self.capture.world_t_camera.orientation.w = 1
    board.get_value_any.return_value = self.packed(
      record_cycle_snapshot(self.context, self.params, self.Result)
    )
    with mock.patch(
      "tools.pose_estimation.cycle_overlay.publish_snapshot",
      side_effect=OSError("ROS unavailable"),
    ):
      self.overlay.publish_new(board)

  def test_observer_keeps_execution_errors_and_exports_final_snapshot(self):
    executive = self.overlay.solution.executive
    result = record_cycle_snapshot(self.context, self.params, self.Result)
    executive.operation.blackboard.get_value_any.return_value = self.packed(
      result
    )
    executive.start.side_effect = RuntimeError("pick failed")
    with mock.patch(
      "tools.pose_estimation.cycle_overlay.publish_snapshot"
    ) as publish:
      with self.assertRaisesRegex(RuntimeError, "pick failed"):
        self.overlay.run("tree", "simulation-mode")
    executive.load.assert_called_once_with("tree")
    executive.start.assert_called_once_with(simulation_mode="simulation-mode")
    publish.assert_called_once()


if __name__ == "__main__":
  unittest.main()
