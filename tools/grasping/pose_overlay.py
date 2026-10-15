"""Read-only RViz CAD/pose snapshot overlay; requires an external ROS 2 Python.

Run as `python -m tools.grasping.pose_overlay` from the repository root in the
ROS environment. It only publishes visualization markers, never TF, robot
commands, perception requests, or belief-world updates.
"""

import argparse
import fcntl
import hashlib
import json
from pathlib import Path

from src.grasping.geometry import RigidTransform, add, scale
from src.grasping.types import TriangleMesh


def marker_specs(cad, diagnostic, fixed_frame="root"):
  """Transform exact object-local CAD and axis endpoints into world metres.

  Snapshot rotations are basis COLUMNS. World-space points with identity marker
  poses avoid quaternion conventions and applying the object transform twice.
  `fixed_frame` must name the diagnostic's world root, not an arbitrary TF frame.
  """
  if not isinstance(fixed_frame, str) or not fixed_frame.strip():
    raise ValueError("fixed_frame must name the snapshot world root.")
  mesh = TriangleMesh(
    tuple(tuple(v) for v in cad["vertices"]),
    tuple(tuple(t) for t in cad["triangles"]),
  )
  specs = []
  snapshot = diagnostic["finished_at_utc"]
  objects = []
  actual = diagnostic.get("actual_after_evaluation_only")
  if actual is not None:
    objects.append(("actual", actual, (0.1, 1.0, 0.2, 0.45), -0.12, 0, ""))
  estimates = diagnostic.get("perceived_objects")
  if estimates is None:
    estimates = (
      [dict(pose=diagnostic["fresh_world_object"])]
      if "fresh_world_object" in diagnostic
      else []
    )
  for index, estimate in enumerate(estimates):
    detail = (
      f"\nDetection {index + 1}: {estimate.get('id', '')}"
      if "perceived_objects" in diagnostic
      else ""
    )
    objects.append(
      (
        "perceived",
        estimate["pose"],
        (1.0, 0.1, 0.8, 0.45),
        0.12 + index * 0.05,
        index * 5,
        detail,
      )
    )
  for name, value, color, label_offset, id_offset, detail in objects:
    pose = RigidTransform(
      tuple(tuple(axis) for axis in value["rotation_basis_columns"]),
      tuple(value["translation_m"]),
    )
    namespace = "omts_pose_snapshot/" + name
    base = dict(frame_id=fixed_frame, namespace=namespace)
    specs.append(
      dict(
        **base,
        id=id_offset,
        type="TRIANGLE_LIST",
        color=color,
        scale=(1.0, 1.0, 1.0),
        points=[
          pose.apply(mesh.vertices[i])
          for triangle in mesh.triangles
          for i in triangle
        ],
      )
    )
    for axis_id, (axis, rgb) in enumerate(
      zip(
        pose.rotation,
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.3, 1.0)),
        strict=True,
      ),
      1,
    ):
      specs.append(
        dict(
          **base,
          id=id_offset + axis_id,
          type="ARROW",
          color=(*rgb, 1.0),
          scale=(0.003, 0.007, 0.012),
          points=[pose.translation, add(pose.translation, scale(axis, 0.09))],
        )
      )
    specs.append(
      dict(
        **base,
        id=id_offset + 4,
        type="TEXT_VIEW_FACING",
        color=(*color[:3], 1.0),
        scale=(0.0, 0.0, 0.012),
        position=add(pose.translation, (0.0, label_offset, 0.06)),
        text=f"{name.upper()} SNAPSHOT\n{snapshot}{detail}\nRGB axes: X/Y/Z",
      )
    )
  return specs


def to_ros(specs, clear_frame=None):
  """ROS imports stay at the visualization boundary, outside numeric tests."""
  from geometry_msgs.msg import Point
  from visualization_msgs.msg import Marker, MarkerArray

  message = MarkerArray()
  if clear_frame is not None:
    # A complete snapshot replaces earlier detections, even after a restart.
    clear = Marker()
    clear.header.frame_id = clear_frame
    clear.pose.orientation.w = 1.0
    clear.action = Marker.DELETEALL
    message.markers.append(clear)
  for spec in specs:
    marker = Marker()
    marker.header.frame_id = spec["frame_id"]
    # Zero stamp requests latest TF, not wall time in a simulation-clock graph.
    marker.ns, marker.id = spec["namespace"], spec["id"]
    marker.type = getattr(Marker, spec["type"])
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    if "position" in spec:
      marker.pose.position = Point(
        **dict(zip("xyz", spec["position"], strict=True))
      )
    marker.scale.x, marker.scale.y, marker.scale.z = spec["scale"]
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec[
      "color"
    ]
    marker.points = [
      Point(**dict(zip("xyz", p, strict=True))) for p in spec.get("points", [])
    ]
    marker.text = spec.get("text", "")
    message.markers.append(marker)
  return message


def split_marker_specs(specs):
  """Keep each pose's CAD, axes, and label together on its own topic."""
  return {
    name: [s for s in specs if s["namespace"] == "omts_pose_snapshot/" + name]
    for name in ("actual", "perceived")
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--cad", type=Path)
  parser.add_argument("--diagnostic", type=Path)
  parser.add_argument(
    "--watch", type=Path, help="Reload an atomic CAD/pose bundle."
  )
  parser.add_argument("--fixed-frame", default="root")
  parser.add_argument("--actual-topic", default="/omts/actual_pose_snapshot")
  parser.add_argument(
    "--perceived-topic", default="/omts/perceived_pose_snapshot"
  )
  args = parser.parse_args()
  if args.actual_topic == args.perceived_topic:
    parser.error("Actual and perceived topics must be different.")
  if not args.watch and not (args.cad and args.diagnostic):
    parser.error("Provide --watch or both --cad and --diagnostic.")
  # Avoid competing latched publishers, including repeated Bazel invocations.
  lock_key = hashlib.sha256(
    (args.actual_topic + "\n" + args.perceived_topic).encode()
  ).hexdigest()[:16]
  lock_path = Path("/tmp/opencode") / f"omts_pose_overlay_{lock_key}.lock"
  lock_path.parent.mkdir(exist_ok=True)
  lock = lock_path.open("a")
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    return

  last_content = args.watch.read_bytes() if args.watch else None

  def load_specs():
    if args.watch:
      bundle = json.loads(last_content)
      return marker_specs(bundle["cad"], bundle, args.fixed_frame)
    return marker_specs(
      json.loads(args.cad.read_text()),
      json.loads(args.diagnostic.read_text()),
      args.fixed_frame,
    )

  specs = load_specs()

  import rclpy
  from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
  from visualization_msgs.msg import Marker, MarkerArray

  rclpy.init()
  node = rclpy.create_node("omts_pose_snapshot_overlay")
  qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
  )
  groups = split_marker_specs(specs)
  publications = [
    (
      node.create_publisher(MarkerArray, topic, qos),
      to_ros(groups[name], args.fixed_frame if args.watch else None),
    )
    for name, topic in (
      ("actual", args.actual_topic),
      ("perceived", args.perceived_topic),
    )
  ]

  def publish():
    nonlocal last_content
    if args.watch:
      try:
        content = args.watch.read_bytes()
        if content != last_content:
          bundle = json.loads(content)
          groups = split_marker_specs(
            marker_specs(bundle["cad"], bundle, args.fixed_frame)
          )
          messages = [
            to_ros(groups[name], args.fixed_frame)
            for name in ("actual", "perceived")
          ]
          for index, message in enumerate(messages):
            publications[index] = (publications[index][0], message)
          last_content = content
          node.get_logger().info(f"Loaded snapshot {bundle['finished_at_utc']}")
      except (OSError, ValueError, KeyError, TypeError) as error:
        node.get_logger().error(f"Snapshot refresh rejected: {error}")
    for publisher, message in publications:
      publisher.publish(message)

  node.create_timer(1.0, publish)
  publish()
  node.get_logger().info(
    f"Publishing actual on {args.actual_topic}, perceived on {args.perceived_topic}, "
    f"frame {args.fixed_frame}. "
    "Green=actual; magenta=perceived; RGB=X/Y/Z. Not live, no control commands."
  )
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if rclpy.ok():
      for publisher, message in publications:
        for marker in message.markers:
          marker.action = Marker.DELETE
        publisher.publish(message)
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()
    lock.close()


if __name__ == "__main__":
  main()
