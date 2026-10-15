"""Collector contract with mocked motion, camera and perception service."""

import csv
import json
import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
from intrinsic.math.python import pose3, proto_conversion, rotation3
from intrinsic.perception.proto.v1 import (
  capture_result_pb2,
  pose_estimation_service_pb2,
)

from src.grasping.geometry import RigidTransform
from tools.pose_estimation.run_viewpoint_benchmark import collect
from tools.pose_estimation.viewpoint_benchmark import (
  make_plan,
  record,
  transform,
)


class ViewpointCollectionTest(unittest.TestCase):
  def exercise(self, detections=True, move_failed=False):
    with tempfile.TemporaryDirectory() as temporary:
      root = Path(temporary)
      actual = RigidTransform()
      plan = make_plan(
        actual, RigidTransform(translation=(0.1, 0.0, 0.3)), count=1
      )
      plan.update(
        tool_t_camera=record(RigidTransform()),
        camera_name="camera",
        object_name="part",
        rgb_sensor="1",
        depth_sensor="4",
        tool_object="tool",
        tool_frame="tcp",
      )
      for name, data in (
        ("plan", plan),
        (
          "cad",
          dict(
            vertices=[[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0]],
            triangles=[[0, 1, 2]],
          ),
        ),
        ("provenance", dict(test="mock")),
      ):
        (root / f"{name}.json").write_text(json.dumps(data))
      optical = transform(plan["views"][0]["camera_pose"])

      def sdk(pose):
        return pose3.Pose3(
          rotation3.Rotation3.from_matrix(np.asarray(pose.rotation).T),
          pose.translation,
        )

      camera = mock.Mock(sensor_id_to_name={1: "rgb", 4: "depth"})
      camera.world_t_sensor.return_value = sdk(optical)
      rgb = SimpleNamespace(
        array=np.zeros((100, 100, 3), dtype=np.uint8),
        world_t_sensor=sdk(optical),
        capture_at=datetime.now(UTC),
        intrinsic_matrix=np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]]),
      )
      capture = SimpleNamespace(
        proto=capture_result_pb2.CaptureResult(),
        sensor_images={
          "rgb": rgb,
          "depth": SimpleNamespace(array=np.ones((100, 100), dtype=np.float32)),
        },
      )
      camera.capture.return_value = capture
      solution = mock.Mock(is_simulated=True)
      solution.resources = {"service": mock.Mock()}
      solution.executive.has_operation = False
      solution.executive.operation.done = True
      solution.executive.operation.proto.HasField.return_value = move_failed
      solution.world.get_transform.return_value = sdk(optical)
      response = pose_estimation_service_pb2.RunPoseEstimationResponse()
      if detections:
        response.pose_estimates.add(
          id="part",
          score=-26,
          camera_t_target=proto_conversion.pose_to_proto(
            sdk(optical.inverse().compose(actual))
          ),
        )
      args = SimpleNamespace(
        execute_simulation=True,
        plan=root / "plan.json",
        cad=root / "cad.json",
        provenance=root / "provenance.json",
        output=root / "run",
        label="test",
        address="unused",
        arm_part="ur_module",
        service="service",
        repeats=1,
        settle_seconds=0,
        estimate_frame="camera",
        estimator="ai.intrinsic.part",
        timeout=30,
      )
      module = "tools.pose_estimation.run_viewpoint_benchmark"
      with (
        mock.patch(module + ".connect", return_value=(solution, mock.Mock())),
        mock.patch(module + ".camera_client", return_value=camera),
        mock.patch(module + ".actual_pose", return_value=actual),
        mock.patch("grpc.intercept_channel"),
        mock.patch(
          "intrinsic.perception.service.python.pose_estimation_client_utils.get_connection_params"
        ),
        mock.patch(
          "intrinsic.perception.service.python.pose_estimation_client_utils.run_pose_estimation_request_from_capture_results",
          return_value=pose_estimation_service_pb2.RunPoseEstimationRequest(),
        ) as prepare,
        mock.patch(
          "intrinsic.perception.proto.v1.pose_estimation_service_pb2_grpc.PoseEstimationServiceStub"
        ) as stub,
        mock.patch("src.hardware.robot.Robot"),
      ):
        stub.return_value.RunPoseEstimation.return_value = response
        if move_failed:
          with self.assertRaisesRegex(RuntimeError, "move failed"):
            collect(args)
          camera.capture.assert_not_called()
          stub.return_value.RunPoseEstimation.assert_not_called()
        else:
          collect(args)
          self.assertEqual(prepare.call_args.args[1], [capture])
          self.assertEqual(
            set(prepare.call_args.kwargs), {"inference_timeout_secs"}
          )
          saved = root / "run/view_000_000"
          for name in (
            "rgb.png",
            "depth.npy",
            "capture.pb",
            "request.pb",
            "response.pb",
            "comparison.png",
          ):
            self.assertTrue((saved / name).is_file())
      with (root / "run/results.csv").open(newline="") as stream:
        return list(csv.DictReader(stream))

  def test_capture_and_received_pose_archived_without_truth_prior(self):
    rows = self.exercise()
    self.assertEqual(len(rows), 1)
    self.assertEqual(rows[0]["status"], "ok")
    self.assertLess(float(rows[0]["position_error_mm"]), 1e-8)
    self.assertEqual(rows[0]["score"], "-26.0")

  def test_no_detection_not_dropped(self):
    rows = self.exercise(detections=False)
    self.assertEqual(rows[0]["status"], "no_detection")
    self.assertEqual(rows[0]["detection_count"], "0")

  def test_move_failure_recorded_and_aborts_before_capture(self):
    rows = self.exercise(move_failed=True)
    self.assertEqual(rows[0]["status"], "move_failed")


if __name__ == "__main__":
  unittest.main()
