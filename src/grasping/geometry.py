"""Small numeric rigid-transform contract; rotations store basis COLUMNS."""

import math
from dataclasses import dataclass

from src.grasping.types import Vec3, _require_tuple, _require_vec3

Rotation = tuple[Vec3, Vec3, Vec3]
IDENTITY: Rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ROTATION_TOLERANCE = 1e-9


def add(a: Vec3, b: Vec3) -> Vec3:
  return tuple(x + y for x, y in zip(a, b, strict=True))


def sub(a: Vec3, b: Vec3) -> Vec3:
  return tuple(x - y for x, y in zip(a, b, strict=True))


def scale(a: Vec3, value: float) -> Vec3:
  return tuple(x * value for x in a)


def dot(a: Vec3, b: Vec3) -> float:
  return math.fsum(x * y for x, y in zip(a, b, strict=True))


def cross(a: Vec3, b: Vec3) -> Vec3:
  return (
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  )


def unit(a: Vec3) -> Vec3:
  length = math.hypot(*a)
  if not math.isfinite(length) or length == 0:
    raise ValueError("direction must have positive finite length.")
  return scale(a, 1 / length)


@dataclass(frozen=True)
class RigidTransform:
  """parent_T_child in metres, with a proper orthonormal column-basis rotation.

  No quaternion ordering or SDK type is implicit. Tuples are required, not
  coerced. Composition returns parent_T_grandchild; apply transforms a point.
  """

  rotation: Rotation = IDENTITY
  translation: Vec3 = (0.0, 0.0, 0.0)

  def __post_init__(self) -> None:
    _require_tuple(self.rotation, "rotation", length=3)
    _require_vec3(self.translation, "translation")
    for axis in self.rotation:
      _require_vec3(axis, "rotation axis")
    for i in range(3):
      for j in range(3):
        if not math.isclose(
          dot(self.rotation[i], self.rotation[j]),
          float(i == j),
          rel_tol=0,
          abs_tol=ROTATION_TOLERANCE,
        ):
          raise ValueError("rotation must be orthonormal.")
    if not math.isclose(
      dot(cross(self.rotation[0], self.rotation[1]), self.rotation[2]),
      1.0,
      rel_tol=0,
      abs_tol=ROTATION_TOLERANCE,
    ):
      raise ValueError("rotation must be right-handed (determinant +1).")

  def rotate(self, direction: Vec3) -> Vec3:
    return tuple(
      math.fsum(
        axis[i] * coordinate
        for axis, coordinate in zip(self.rotation, direction, strict=True)
      )
      for i in range(3)
    )

  def apply(self, point: Vec3) -> Vec3:
    return add(self.rotate(point), self.translation)

  def compose(self, child_T_grandchild: "RigidTransform") -> "RigidTransform":
    return RigidTransform(
      tuple(self.rotate(axis) for axis in child_T_grandchild.rotation),
      self.apply(child_T_grandchild.translation),
    )

  def inverse(self) -> "RigidTransform":
    transposed = tuple(
      tuple(axis[i] for axis in self.rotation) for i in range(3)
    )
    return RigidTransform(
      transposed, tuple(-dot(axis, self.translation) for axis in self.rotation)
    )
