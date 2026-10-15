"""Evaluation-only snapshots from a running cycle; ROS stays on the host."""

import inspect
import logging
import threading

import grpc
from intrinsic.math.python import proto_conversion
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb

from tools.pose_estimation.pose_snapshot import make_snapshot, publish_snapshot


def record_cycle_snapshot(context, params, result_type):
  """Runs after inference, before motion; never updates either world."""
  from datetime import datetime, timezone
  from intrinsic.world.proto import object_world_service_pb2

  result = result_type()
  try:
    if not params.capture.HasField("world_t_camera"):
      raise ValueError(
        "Capture has no world_t_camera; overlay cannot use root."
      )
    result.capture.ParseFromString(params.capture.SerializeToString())
    for estimate in params.estimates:
      result.estimates.add().ParseFromString(estimate.SerializeToString())
    # shortcut: truth is sampled after inference, use synchronized history if
    # the workpiece can move between exposure and this pre-pick observation.
    request = object_world_service_pb2.GetTransformRequest(world_id="sim_world")
    request.node_a.by_name.object.object_name = "root"
    request.node_b.by_name.object.object_name = params.object_name
    response = context.object_world.stub.GetTransform(request, timeout=1.0)
    result.actual.ParseFromString(response.a_t_b.SerializeToString())
  except Exception as error:
    # Optional evaluation must not fail perception, cause retries, or substitute
    # belief poses for simulator truth. The host reports this diagnostic.
    result.warning = str(error)
  result.finished_at_utc = datetime.now(timezone.utc).isoformat()
  return result


def _pose_record(proto):
  pose = proto_conversion.pose_from_proto(proto)
  return dict(
    translation_m=[float(x) for x in pose.translation],
    rotation_basis_columns=[
      [float(x) for x in pose.rotation.rotate_point(axis)]
      for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    ],
  )


class CyclePoseOverlay:
  """Owns diagnostic task results and their read-only host publication."""

  def __init__(
    self,
    solution,
    object_name,
    estimator_id,
    cad_path,
    workspace,
    manifest=None,
  ):
    self.solution = solution
    self.object_name = object_name
    self.estimator_id = estimator_id
    self.cad_path = cad_path
    self.workspace = workspace
    self.manifest = manifest
    self.results = []
    self._seen = {}

  def build_task(self, capture, estimate):
    signature = self.solution.proto_builder.create_signature_with_args(
      parameters=pb.MessageSpec(
        fields=[
          pb.FieldSpec(
            type="intrinsic_proto.perception.v1.CaptureData",
            name="capture",
            number=1,
            arg=capture.result.capture_data,
          ),
          pb.FieldSpec(
            type="intrinsic_proto.perception.v1.PoseEstimateInRoot",
            name="estimates",
            number=2,
            repeated=True,
            arg=estimate.result.estimates,
          ),
          pb.FieldSpec(
            type="string", name="object_name", number=3, arg=self.object_name
          ),
        ]
      ),
      return_value=pb.MessageSpec(
        fields=[
          pb.FieldSpec(
            type="intrinsic_proto.perception.v1.CaptureData",
            name="capture",
            number=1,
          ),
          pb.FieldSpec(
            type="intrinsic_proto.perception.v1.PoseEstimateInRoot",
            name="estimates",
            number=2,
            repeated=True,
          ),
          pb.FieldSpec(type="intrinsic_proto.Pose", name="actual", number=3),
          pb.FieldSpec(type="string", name="finished_at_utc", number=4),
          pb.FieldSpec(type="string", name="warning", number=5),
        ]
      ),
    )
    script = bt.PythonScript(
      signature_with_args=signature,
      function_body=inspect.getsource(record_cycle_snapshot)
      + "\nreturn record_cycle_snapshot(context, params, node_pb2.ReturnValue)",
    )
    self.results.append(script.result)
    # Code-execution infrastructure failures are optional too, not a reason to
    # repeat inference or prevent a pick. Normal failures return a warning above.
    return bt.Fallback(
      name="Record Evaluation Pose Snapshot",
      children=[
        bt.Task(
          action=script, name="Record Captured Estimate & Simulator Pose"
        ),
        bt.Sequence(name="Skip Unavailable Pose Overlay", children=[]),
      ],
    )

  def publish_new(self, blackboard, stop=None):
    for index, key in enumerate(self.results):
      try:
        # Decode using the signature already built on this host rather than
        # relying on the runtime proto registry to know the dynamic result.
        result = key.value_type()
        if not blackboard.get_value_any(key).Unpack(result):
          raise ValueError(
            "Cycle snapshot result has an unexpected proto type."
          )
        if stop is not None and stop.is_set():
          return
        if not result.finished_at_utc or (
          self._seen.get(index) == result.finished_at_utc
        ):
          continue
        self._seen[index] = result.finished_at_utc
        if result.warning:
          logging.warning("Cycle pose overlay: %s", result.warning)
        if not result.capture.HasField("world_t_camera"):
          continue
        snapshot = make_snapshot(
          _pose_record(result.capture.world_t_camera),
          [
            dict(id=e.id, score=e.score, pose=_pose_record(e.root_t_target))
            for e in result.estimates
          ],
          _pose_record(result.actual) if result.HasField("actual") else None,
          result.finished_at_utc,
          "camera",
        )
        snapshot["pose_estimator_id"] = self.estimator_id
        publish_snapshot(snapshot, self.cad_path, self.workspace, self.manifest)
        logging.info("Cycle pose overlay refreshed: %s", result.finished_at_utc)
      except grpc.RpcError as error:
        if error.code() != grpc.StatusCode.NOT_FOUND:
          logging.warning("Cycle pose overlay unavailable: %s", error)
      except Exception as error:
        logging.warning("Cycle pose overlay unavailable: %s", error)

  def run(self, tree, simulation_mode):
    executive = self.solution.executive
    executive.load(tree)
    # Pin reads to this operation, never a previous or replacement execution.
    blackboard = executive.operation.blackboard
    stop = threading.Event()

    def observe():
      while not stop.is_set():
        self.publish_new(blackboard, stop)
        stop.wait(0.1)
      # Include a last estimate even when execution finished/failed quickly.
      # Keep the final read on this daemon thread, not the control caller.
      self.publish_new(blackboard)

    worker = threading.Thread(target=observe, daemon=True)
    worker.start()
    try:
      executive.start(simulation_mode=simulation_mode)
    finally:
      stop.set()
      worker.join(timeout=2.0)
      if worker.is_alive():
        logging.warning("Pose overlay observer is still awaiting its read RPC.")
