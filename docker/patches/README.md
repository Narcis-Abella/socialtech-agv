# Patches to third-party sources

Applied at image build, after fetching `robot.repos`: `patches/<repo>/*.patch` in lexical order,
where `<repo>` is the key in `robot.repos`. A patch that no longer applies fails the build.

| Change type | Goes where |
|---|---|
| Fix needed to build/run upstream code (e.g. C++ standard) | here, as a `.patch` |
| Our setup's configuration (params, IPs, launch args) | own bringup package — never patch upstream configs |
| New logic/features in upstream code | fork on GitHub, pin the fork commit in `robot.repos` |

Create a patch against the pinned commit:

```bash
git init x && cd x
git fetch --depth 1 <url> <commit> && git checkout FETCH_HEAD
# edit files
git diff > ../patches/<repo>/NNNN-short-description.patch
```

Put a short "why" and the base commit at the top of the file (text before `diff --git` is ignored).
When bumping a commit in `robot.repos`, re-check each patch for that repo — drop it if upstream fixed it.
