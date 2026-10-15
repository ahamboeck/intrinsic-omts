"""Project exact CAD into captured RGB; drawing is evaluation-only."""

import numpy as np
from PIL import Image, ImageDraw


def cad_in_image(shape, intrinsics, world_camera, actual, cad):
  """Frustum test only: it cannot detect occlusion by the gripper or enclosure."""
  camera_object = world_camera.inverse().compose(actual)
  for vertex in cad["vertices"]:
    point = camera_object.apply(tuple(vertex))
    if point[2] <= 0:
      return False
    uv = np.asarray(intrinsics) @ np.asarray(point)
    u, v = uv[:2] / uv[2]
    if not (0 <= u < shape[1] and 0 <= v < shape[0]):
      return False
  return True


def comparison_image(rgb, intrinsics, world_camera, actual, detections, cad):
  """Two panels of the same RGB image, actual vs all received detections.

  RGB is captured with undistortion enabled; projection uses its returned K.
  Behind-camera vertices are not projected. This is not an occlusion renderer.
  """
  image = Image.fromarray(rgb.astype(np.uint8)).convert("RGB")
  panels = [image.copy(), image.copy()]
  inverse = world_camera.inverse()
  edges = sorted(
    {
      tuple(sorted((a, b)))
      for t in cad["triangles"]
      for a, b in zip(t, (t[1], t[2], t[0]), strict=True)
    }
  )

  def draw_mesh(panel, pose, color):
    camera_object = inverse.compose(pose)
    points = []
    for vertex in cad["vertices"]:
      point = camera_object.apply(tuple(vertex))
      if point[2] <= 0:
        points.append(None)
      else:
        uv = np.asarray(intrinsics) @ np.asarray(point)
        points.append((float(uv[0] / uv[2]), float(uv[1] / uv[2])))
    draw = ImageDraw.Draw(panel)
    for a, b in edges:
      if points[a] is not None and points[b] is not None:
        draw.line([points[a], points[b]], fill=color, width=2)

  draw_mesh(panels[0], actual, (0, 255, 70))
  for pose in detections:
    draw_mesh(panels[1], pose, (255, 40, 210))
  for panel, label in zip(
    panels,
    ("GROUND TRUTH", "DETECTIONS" if detections else "NO DETECTION"),
    strict=True,
  ):
    draw = ImageDraw.Draw(panel)
    draw.rectangle((0, 0, 180, 24), fill="black")
    draw.text((5, 5), label, fill="white")
  output = Image.new("RGB", (image.width * 2, image.height))
  output.paste(panels[0], (0, 0))
  output.paste(panels[1], (image.width, 0))
  return output
