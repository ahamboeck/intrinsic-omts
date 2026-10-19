"""Exercise the deployed wrapper with invalid depth pixels and fake networks."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


class Tensor:
  def __init__(self, name, array):
    self.name = name
    self.array = array

  def as_numpy(self):
    return self.array


class FoundationPoseDepthTest(unittest.TestCase):
  def setUp(self):
    cpp = types.SimpleNamespace(
      update_refined_poses=lambda poses, *args: poses,
      apply_mesh_center_offset=lambda pose, center: pose,
    )
    pb = types.SimpleNamespace(
      get_input_tensor_by_name=lambda request, name: request.get(name),
      Tensor=Tensor,
      InferenceResponse=lambda **kwargs: types.SimpleNamespace(**kwargs),
      TritonError=lambda message: message,
    )
    path = Path(__file__).parents[2] / "third_party/foundationpose/model.py"
    spec = importlib.util.spec_from_file_location("depth_test_model", path)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(
      sys.modules,
      {
        "cv2": types.SimpleNamespace(),
        "foundationpose_cpp": cpp,
        "onnxruntime": types.SimpleNamespace(),
        "trimesh": types.SimpleNamespace(Trimesh=object),
        "triton_python_backend_utils": pb,
      },
    ):
      spec.loader.exec_module(module)
    self.model = module.TritonPythonModel()
    self.model._load_glb_mesh = lambda data: types.SimpleNamespace(
      vertices=np.array([[-0.01, -0.01, -0.01], [0.01, 0.01, 0.01]]),
      faces=np.array([[0, 1, 0]]),
    )
    self.sampled_depth = []
    self.xyz_maps = []

    def sample(mask, depth, *args, **kwargs):
      self.sampled_depth.append(depth.copy())
      return np.eye(4, dtype=np.float32)[None]

    def prepare(poses, K, vertices, faces, diameter, xyz_map, rgb):
      self.xyz_maps.append(xyz_map.copy())
      return np.zeros((len(poses), 1)), np.zeros((len(poses), 1))

    self.model._sample_initial_poses = sample
    self.model._prepare_model_inputs = prepare
    self.model.refine_session = types.SimpleNamespace(
      run=lambda outputs, inputs: (np.zeros((1, 3)), np.zeros((1, 3)))
    )
    self.model.score_session = types.SimpleNamespace(
      run=lambda outputs, inputs: [np.ones((1, 1))]
    )

  def test_invalid_depth_becomes_zero_before_inference(self):
    depth = np.array(
      [[0.3, np.inf, -np.inf], [np.nan, -0.2, 0.0]], dtype=np.float32
    )
    request = {
      "RGB": Tensor("RGB", np.zeros((2, 3, 3), dtype=np.uint8)),
      "DEPTH": Tensor("DEPTH", depth),
      "MASK": Tensor("MASK", np.ones((1, 2, 3), dtype=np.uint8)),
      "CAM_K": Tensor("CAM_K", np.eye(3, dtype=np.float32)),
      "CAD_MODEL_BYTES": Tensor("CAD_MODEL_BYTES", np.zeros(1, dtype=np.uint8)),
      "NUM_ITERATIONS": Tensor("NUM_ITERATIONS", np.array([1])),
      "BATCH_SIZE": Tensor("BATCH_SIZE", np.array([1])),
    }
    response = self.model.execute([request])[0]
    self.assertFalse(hasattr(response, "error"), getattr(response, "error", ""))
    expected = np.array([[0.3, 0, 0], [0, 0, 0]], dtype=np.float32)
    np.testing.assert_array_equal(self.sampled_depth[0], expected)
    self.assertTrue(self.xyz_maps)
    for xyz_map in self.xyz_maps:
      self.assertTrue(np.all(np.isfinite(xyz_map)))
      np.testing.assert_array_equal(xyz_map[..., 2], expected)


if __name__ == "__main__":
  unittest.main()
