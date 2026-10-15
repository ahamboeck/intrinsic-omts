# Local Core integration

This is a development integration, not a replacement of the installed Core base
runtime. Each checkout's `MODULE.bazel` selects its local Core checkout and its
`intrinsic_apis` subdirectory directly, so commands on these branches do not need
CLI overrides.
The working-tree contents of both repositories are build inputs, including
uncommitted changes.

## Source pairing

| OMTS checkout / branch | Local Core path / branch |
| --- | --- |
| `repos/intrinsic-omts` / `integration/local-core-upgrade` | `../../.local/core-omts-integration` / `integration/omts-build-compatibility` |
| `repos/intrinsic-omts-modified` / `comparison/omts-modified` | `../intrinsic-core-modified` / `comparison/core-modified` |

OMTS is based on `feature/mesh-grasp-demo` at `01a61b6`; Core is based on
`fix/ioc-rgb-depth-registration` at `0221644` with the existing RGB-D edits.
The OMTS source trees were synchronized; their Core paths and build outputs
remain separate. Future edits are not automatically synchronized.

- Core PR checkout: `repos/intrinsic-core` on `fix/ioc-rgb-depth-registration`,
  containing only the five RGB-D bugfix files, not the integration workarounds.
- Bazel: 8.8.1, selected by `.bazelversion` when using Bazelisk.

Existing uncommitted mesh and RGB-D changes are retained. Branch creation alone
does not save them in Git history. Before switching branches, preserve the
changes in commits or separate worktrees; do not assume a branch switch removes
uncommitted integration edits. The RGB-D files were copied into the integration
worktree; future PR edits are not automatically synchronized to that copy.

## Compatibility changes

- Core's unavailable, unused `bazel/js_rules.bzl` extension registration is
  removed from its module configuration.
- The Go Gazelle override uses `github.com/googlecloudrobotics/core`, matching
  the newer Core `go.mod`.
- Zenoh 1.8.0.bcr.1 implements static linking when `--dynamic_mode=off`, already
  set by OMTS. The obsolete 1.7.2 patch is no longer registered.
- FlatBuffers' module is patched to omit `root_package`, removed in rules_js 3.
  The package is inferred as `ts` from `//ts:pnpm-lock.yaml` instead.
- The existing Robotiq finger offsets are applied directly in the Core
  integration checkout, because local overrides do not apply archive patches.
- OMTS's scene-object rule, asset metadata generator, and asset protobuf test
  dependency use their new `intrinsic_sdk/` target locations.
- The pose-registration and estimation tools (and registration test) use the
  relocated `intrinsic_sdk/` asset ID and installed-assets protobuf targets.
- The estimation tool and application adapter pass camera data through
  `capture_data`; the newer multi-view estimation skill no longer accepts
  `camera_1`–`camera_4` slots.

## Build-only checks

Run from `repos/intrinsic-omts`:

```bash
git -C ../../.local/core-omts-integration status --short --branch
env -u LD_PRELOAD bazel mod show_repo --lockfile_mode=off @intrinsic-core

env -u LD_PRELOAD bazel test --lockfile_mode=off --jobs=4 \
  --local_resources=memory=10000 --test_output=errors \
  @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:depth_registration_test \
  @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:depth_registration_input_test

env -u LD_PRELOAD bazel build --lockfile_mode=off --jobs=4 \
  --local_resources=memory=10000 --//:setup=lab_bb_01 //:omts_solution
```

For isolated default-OMTS comparison checks, run from
`intrinsic/.local/comparison` instead:

```bash
bash build.sh modified resolve
bash build.sh modified test
bash build.sh modified build
```

This helper targets `repos/intrinsic-omts-modified` and uses its separate output
base; it does not deploy or run the application.

`--lockfile_mode=off` avoids rewriting the published-release lockfile during this
integration trial. A final reproducible integration should generate and review
its own lockfile after the dependency configuration stabilizes.

Core's model assets must be materialized rather than Git LFS pointer files:

From `repos/intrinsic-omts`, use the path below. From
`repos/intrinsic-omts-modified`, substitute `../intrinsic-core-modified`.

```bash
git -C ../../.local/core-omts-integration lfs pull origin \
  --include='intrinsic/models/assets/**,intrinsic_control/intrinsic/models/assets/**,intrinsic_control/intrinsic/models/robot_definitions/**,third_party/**' \
  --exclude=''
```

These commands do not deploy or stop any service. A successful build does not
prove compatibility with the installed base runtime. Deployment is a separate,
explicit step, and must specify `--operation_mode=sim` for simulation. Reapply
scene updates and register the estimator after redeployment as described in the
main README.

To return to published Core sources, use the original OMTS branch/checkouts
without local overrides, after saving uncommitted changes. Do not use a hard
reset or discard existing work.
