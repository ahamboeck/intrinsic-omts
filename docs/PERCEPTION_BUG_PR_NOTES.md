# Two separate perception bug PRs

Neither fix is deployed. No motion is required to reproduce either bug.
Grasp approach tilt and demo changes are separate OMTS work, not part of these
two perception PRs. Do not bundle all current working-tree changes together.

## PR 1 — OMTS: score a complete FoundationPose candidate set

**Owner/repository:** `intrinsic-omts`.

**Files:** `third_party/foundationpose/model.py`,
`tests/unit/test_foundationpose_scoring.py`, its `tests/unit/BUILD` target,
and the candidate-scoring section of `src/foundationpose/README.md`.

The wrapper refined and scored 280 hypotheses as 128 + 128 + 24, comparing
logits from independent score-network calls. Scores depend on the candidate
set; the short last batch had a different baseline and unfairly won.

Controlled reproduction held all 280 refined poses and score input tensors
fixed. Split scoring picked candidate 276 (+6.34), with about 83 degrees axis
error and 11.3 mm position error against simulator evaluation state. Joint
scoring picked candidate 84 (-20.28), with about 0.8 degrees and 1.6 mm error.
Candidate 276 scored -20.78 in the complete set. Simulator state was only used
to evaluate the output, not to select or initialize poses.

Fix: retain batched refinement/rendering, concatenate inputs, score once per
object. Reject empty candidate sets and malformed/nonfinite scores. Full-set
scoring increases peak memory compared with scoring each chunk separately.
NVIDIA's pinned reference pipeline also accumulates candidates for joint scoring.
This is a wrapper integration bug, not evidence of a defect in NVIDIA's weights.

```sh
bazel test --test_output=errors //tests/unit:test_foundationpose_scoring
bazel build //src/foundationpose:foundationpose_mlmodel
```

Recorded GPU diagnostics: `/tmp/opencode/omts_isolated_scoring.json` and
`/tmp/opencode/omts_patched_wrapper_result.json` (local evidence, not PR fixtures).

## PR 2 — Core: register native depth into the RGB image grid

**Owner/repository:** sibling `intrinsic-core`.

**Files under** `intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/`:
`service/depth_registration.py`, `service/depth_registration_test.py`,
`service/depth_registration_input_test.py`,
`service/ioc_pose_estimator_service.py`, and `BUILD`.

The service used native depth pixels with RGB intrinsics without registration.
Different camera intrinsics are legitimate; interpreting their pixels as already
aligned is not. Lab BB-01's simulated Orbbec returns co-located 1280x800 images
with RGB fx/fy 597/593 and depth fx/fy 640/628. Source and actual pixels both
confirm separate native projections, not already registered depth.

Raw depth foreground/RGB segmentation IoU was 84.7%; explicit registration
raised it to 95.0%. With joint scoring held constant, position error improved
from 1.64 to 0.87 mm and largest unsigned axis error from 0.77 to 0.40 degrees.
This is one stationary recorded case, not a hardware-wide accuracy guarantee.
The original 83-degree failure was dominated by PR 1, not this issue.

Fix at the input boundary: use both intrinsic matrices and
`RGB_T_depth = inverse(camera_T_RGB) * camera_T_depth`. Co-located sensors use
inverse nearest-neighbour resampling; separated sensors reproject measured
3D points and z-buffer the nearest positive RGB-frame axial Z. Missing pixels
remain zero. Already aligned matching calibration is preserved. Invalid depth
is removed. Missing calibration and nonzero distortion fail explicitly; this
implementation requires undistorted pinhole images, rather than silently
inventing calibration or depth scale. Depth units remain metres.

```sh
# Run from intrinsic-core:
bazel test --test_output=errors \
  //intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:depth_registration_test \
  //intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:depth_registration_input_test
```

Recorded diagnostics: `/tmp/opencode/omts_pixel_alignment_result.json` and
`/tmp/opencode/omts_registered_fixed_scoring_evaluation.json`.

The Core checkout is not OMTS's pinned external dependency. A Core PR alone
does not change OMTS builds or the deployed service: rebuild/deploy the service
and update the pin (or deliberately use an override) in a later integration step.
