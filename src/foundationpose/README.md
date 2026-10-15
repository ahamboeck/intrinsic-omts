# FoundationPose Build & Packaging

This directory contains the Triton Inference Server model configuration (`config.pbtxt`) and `intrinsic_mlmodel` asset definition (`//src/foundationpose:foundationpose_mlmodel`) for FoundationPose (Python 3.12 / CUDA).

## Overview

The FoundationPose deployment asset (`//src/foundationpose:foundationpose_mlmodel`) is built hermetically from source using Bazel:
1. **`//third_party/foundationpose:foundationpose_cpp.so`** *(internal target)*: Compiled via `@rules_cuda` and `@pybind11_bazel` from `//third_party/foundationpose/cpp` and `nvdiffrast` sources from `@isaac_ros_pose_estimation` (automatically transitioned to Python 3.12 when depended upon by `:env_tar_gz`).
2. **`//third_party/foundationpose:env_tar_gz`**: Assembled hermetically from all Python 3.12 wheels locked in `@foundationpose_pip_deps` (`all_whl_requirements`) and `foundationpose_cpp.so` (installed in `site-packages/`).
3. **`//src/foundationpose:foundationpose_mlmodel`**: Packages `config.pbtxt`, `//third_party/foundationpose:model.py`, `//third_party/foundationpose:env_tar_gz`, and the ONNX model weights (`@foundationpose_refine_onnx` and `@foundationpose_score_onnx`) into the Intrinsic MLModel asset.

## Building with Bazel

To build the complete Intrinsic MLModel asset:
```bash
bazel build //src/foundationpose:foundationpose_mlmodel
```

## Candidate batching and scoring

`BATCH_SIZE` bounds refinement and rendering batches, **not scoring**. All
refined candidate inputs for one object are accumulated and scored together.
The score network depends on the candidate set: logits from separate batches
(especially a smaller final batch) cannot be compared to select a global winner.
Joint scoring requires memory for the full, fixed rotation-grid candidate set.
Malformed/nonfinite scores fail inference rather than returning an identity
fallback. The regression runs without CUDA or real weights:

```sh
bazel test --test_output=errors //tests/unit:test_foundationpose_scoring
```

Source changes require rebuilding and redeploying the model asset; unit tests or
recorded-input diagnostic execution do not update an already deployed server.

## Updating Python Dependencies

Python 3.12 dependencies are declared in `//third_party/foundationpose:requirements.in` and locked with SHA-256 hashes in `//third_party/foundationpose:requirements.txt`. To update the lockfile (no `BUILD` edits required):
```bash
bazel run //third_party/foundationpose:requirements
```

## Code Organization & Licensing

- **`//src/foundationpose/`**: Contains Intrinsic-authored Triton configuration (`config.pbtxt`) and `intrinsic_mlmodel` packaging (`BUILD`).
- **`//third_party/foundationpose/`**: Contains all NVIDIA-copyrighted C++/CUDA sources (`cpp/`), Python orchestration code (`model.py`), external dependency module extensions (`deps.bzl`), build overlays (`isaac_ros_pose_estimation.BUILD`, `nvdiffrast.patch`), and Python lockfiles (`requirements.in`, `requirements.txt`), licensed under Apache-2.0.
