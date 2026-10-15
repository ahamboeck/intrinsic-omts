"""Exercise the deployed wrapper with batch-context-sensitive fake networks."""

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


class FoundationPoseScoringTest(unittest.TestCase):
  def setUp(self):
    self.refine_batches = []
    self.score_batches = []
    cpp = types.SimpleNamespace(update_refined_poses=lambda poses, *args: poses)
    self.cpp = cpp
    cpp.apply_mesh_center_offset = lambda pose, center: pose
    pb = types.SimpleNamespace(
      get_input_tensor_by_name=lambda request, name: request.get(name),
      Tensor=Tensor,
      InferenceResponse=lambda **kwargs: types.SimpleNamespace(**kwargs),
      TritonError=lambda message: message,
    )
    path = Path(__file__).parents[2] / "third_party/foundationpose/model.py"
    spec = importlib.util.spec_from_file_location("scoring_test_model", path)
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
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 5, axis=0)
    poses[:, 0, 3] = np.arange(5)
    self.model._sample_initial_poses = lambda *args, **kwargs: poses.copy()
    self.model._prepare_model_inputs = lambda poses, *args: (
      poses[:, 0, 3:4].copy(),
      poses[:, 0, 3:4].copy(),
    )

    def refine(outputs, inputs):
      count = len(inputs["input1"])
      self.refine_batches.append(count)
      return (np.zeros((count, 3)), np.zeros((count, 3)))

    def score(outputs, inputs):
      ids = inputs["input1"][:, 0]
      self.score_batches.append(ids.tolist())
      # Like the real scorer, logits depend on the candidate set. A short final
      # batch has a different baseline; comparing it to earlier batches is wrong.
      return [(5 - ids + (10 if len(ids) == 1 else 0))[:, None]]

    self.model.refine_session = types.SimpleNamespace(run=refine)
    self.model.score_session = types.SimpleNamespace(run=score)

  def execute(self, batch_size, masks=1, expect_error=False):
    request = {
      "RGB": Tensor("RGB", np.zeros((2, 2, 3), dtype=np.uint8)),
      "DEPTH": Tensor("DEPTH", np.ones((2, 2), dtype=np.float32)),
      "MASK": Tensor("MASK", np.ones((masks, 2, 2), dtype=np.uint8)),
      "CAM_K": Tensor("CAM_K", np.eye(3, dtype=np.float32)),
      "CAD_MODEL_BYTES": Tensor("CAD_MODEL_BYTES", np.zeros(1, dtype=np.uint8)),
      "NUM_ITERATIONS": Tensor("NUM_ITERATIONS", np.array([1])),
      "BATCH_SIZE": Tensor("BATCH_SIZE", np.array([batch_size])),
    }
    response = self.model.execute([request])[0]
    if expect_error:
      return response
    self.assertFalse(hasattr(response, "error"), getattr(response, "error", ""))
    return {
      tensor.name: tensor.as_numpy() for tensor in response.output_tensors
    }

  def test_partial_final_refinement_batch_does_not_bias_scoring(self):
    result = self.execute(2)
    self.assertEqual(self.refine_batches, [2, 2, 1])
    self.assertEqual(self.score_batches, [[0, 1, 2, 3, 4]])
    np.testing.assert_array_equal(result["TRANSLATION"], [[0, 0, 0]])
    np.testing.assert_array_equal(result["CONFIDENCE"], [[5]])

  def test_scoring_is_independent_of_refinement_batch_size(self):
    results = [self.execute(size) for size in (2, 3, 5, 8, 0)]
    for result in results[1:]:
      for name in ("ROTATION", "TRANSLATION", "CONFIDENCE"):
        np.testing.assert_array_equal(result[name], results[0][name])

  def test_selected_pose_includes_refinement(self):
    def refine_poses(poses, *args):
      poses = poses.copy()
      poses[:, 1, 3] = 0.25
      return poses

    self.cpp.update_refined_poses = refine_poses
    result = self.execute(2)
    np.testing.assert_array_equal(result["TRANSLATION"], [[0, 0.25, 0]])

  def test_each_object_gets_its_own_complete_candidate_set(self):
    result = self.execute(2, masks=2)
    self.assertEqual(self.score_batches, [[0, 1, 2, 3, 4]] * 2)
    self.assertEqual(result["TRANSLATION"].shape, (2, 3))

  def test_invalid_scores_fail_instead_of_returning_fallback_pose(self):
    for scores in (np.full((5, 1), np.nan), np.zeros((4, 1))):
      with self.subTest(scores=scores):
        self.model.score_session.run = lambda *args, scores=scores: [scores]
        response = self.execute(2, expect_error=True)
        self.assertIn("invalid candidate scores", response.error)
        self.assertEqual(response.output_tensors, [])


if __name__ == "__main__":
  unittest.main()
