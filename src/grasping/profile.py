"""Numeric, symmetric prismatic parallel-jaw collision-envelope profiles."""

import math
from dataclasses import dataclass

from src.grasping.geometry import IDENTITY, RigidTransform
from src.grasping.types import (
  ParallelJawLimits,
  Vec3,
  _require_tuple,
  _require_vec3,
)


@dataclass(frozen=True)
class CollisionBox:
  """Oriented enclosing box at maximum opening, expressed in grasp-center axes.

  A contact pad is an envelope, NOT a blanket collision exemption: closure may
  touch only its inward X face, without penetration. All other contact is rejected.
  """

  name: str
  center_T_box: RigidTransform
  half_extents_m: Vec3
  is_contact_pad: bool = False

  def __post_init__(self) -> None:
    if not isinstance(self.name, str) or not self.name:
      raise ValueError("collision box requires a nonempty name.")
    if not isinstance(self.center_T_box, RigidTransform):
      raise ValueError("center_T_box must be a RigidTransform.")
    _require_vec3(self.half_extents_m, "half_extents_m")
    if any(v <= 0 for v in self.half_extents_m):
      raise ValueError("half extents must be positive.")
    if type(self.is_contact_pad) is not bool:
      raise ValueError("is_contact_pad must be bool.")


@dataclass(frozen=True)
class ParallelJawProfile:
  """End-effector only, with fixed palm and jaws translating along +/-X.

  Reducing opening moves each jaw inward by half the change. Center origin is
  the contact midpoint. +Z is approach, NOT necessarily the TCP's +Z. The
  declared center_T_tcp must match the deployment; numeric validity is not
  hardware calibration or proof that these envelopes contain the visual CAD.
  """

  name: str
  limits: ParallelJawLimits
  center_T_tcp: RigidTransform
  body_boxes: tuple[CollisionBox, ...]
  positive_jaw_boxes: tuple[CollisionBox, ...]
  negative_jaw_boxes: tuple[CollisionBox, ...]

  def __post_init__(self) -> None:
    if not isinstance(self.name, str) or not self.name:
      raise ValueError("profile requires a nonempty name.")
    if not isinstance(self.limits, ParallelJawLimits) or not isinstance(
      self.center_T_tcp, RigidTransform
    ):
      raise ValueError("profile requires jaw limits and a rigid TCP transform.")
    names = []
    for boxes in (
      self.body_boxes,
      self.positive_jaw_boxes,
      self.negative_jaw_boxes,
    ):
      _require_tuple(boxes, "collision boxes")
      if not boxes or any(not isinstance(box, CollisionBox) for box in boxes):
        raise ValueError("each profile group requires CollisionBox values.")
      names.extend(box.name for box in boxes)
    if len(set(names)) != len(names):
      raise ValueError("collision box names must be unique.")
    if any(box.is_contact_pad for box in self.body_boxes):
      raise ValueError("body boxes cannot be contact pads.")
    for sign, boxes in (
      (1, self.positive_jaw_boxes),
      (-1, self.negative_jaw_boxes),
    ):
      pads = [box for box in boxes if box.is_contact_pad]
      if len(pads) != 1:
        raise ValueError("each jaw must have exactly one contact-pad envelope.")
      pad = pads[0]
      if pad.center_T_box.rotation != IDENTITY:
        raise ValueError(
          "contact-pad envelopes must be aligned to center axes."
        )
      position = pad.center_T_box.translation
      inner_x = sign * position[0] - pad.half_extents_m[0]
      if not math.isclose(
        inner_x, self.limits.max_opening_m / 2, rel_tol=0, abs_tol=1e-12
      ) or position[1:] != (0, 0):
        raise ValueError(
          "pad inner faces must define the centered maximum opening."
        )
