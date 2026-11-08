# Patches to third-party sources

Applied at image build, after fetching `deps/robot/robot.repos`: `patches/<repo>/*.patch` in lexical order,
where `<repo>` is the key in `robot.repos`. A fix inside a git submodule of that repo goes in `patches/<repo>/<submodule path>/*.patch`
(e.g. `FAST_LIO/include/ikd-Tree/`): `git apply` of the parent cannot reach into it. A patch that no longer applies fails the build.

| Change type | Goes where |
|---|---|
| Fix needed to build/run upstream code (e.g. C++ standard), or a small change that keeps upstream's behaviour by default (a switch, a memory bound, a null guard) | here, as a `.patch` |
| Our setup's configuration (params, IPs, launch args) | `ops/` (overrides and launch files) — never patch upstream configs |
| New logic/features that change what upstream does | fork on GitHub, pin the fork commit in `robot.repos` |

Create a patch against the pinned commit:

```bash
git init x && cd x
git fetch --depth 1 <url> <commit> && git checkout FETCH_HEAD
# edit files
git diff > <repo root>/deps/robot/patches/<repo>/NNNN-short-description.patch
```

Put a short "why" and the base commit at the top of the file (text before `diff --git` is ignored).
When bumping a commit in `robot.repos`, re-check each patch for that repo — drop it if upstream fixed it.

License: a patch is a modification of its upstream code, so it falls under that code's license: GPL-2.0 for FAST-LIO2 and ikd-Tree, the same as this repository (`LICENSE`); `tracer_ros2` keeps the license of its repository.
