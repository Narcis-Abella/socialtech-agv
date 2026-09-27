#!/usr/bin/env python3
"""Shallow-fetch every repo of a vcstool .repos file at its pinned commit.

`vcs import` clones full histories (OrbbecSDK_ROS2 alone is ~800 MB) and cannot shallow-clone
a commit SHA; `git fetch --depth 1 <sha>` downloads only that snapshot.

Usage: fetch_repos.py <file.repos> <dest_dir>
"""
import pathlib
import subprocess
import sys

import yaml


def git(path, *args):
    subprocess.run(["git", "-C", str(path), *args], check=True)


def main():
    repos_file, dest = sys.argv[1], pathlib.Path(sys.argv[2])
    with open(repos_file) as f:
        repos = yaml.safe_load(f)["repositories"]
    for name, spec in repos.items():
        path = dest / name
        path.mkdir(parents=True, exist_ok=True)  # re-runnable: fetch + checkout are idempotent
        print(f"fetching {name} @ {spec['version'][:10]}", flush=True)
        git(path, "init", "-q")
        git(path, "fetch", "-q", "--depth", "1", spec["url"], spec["version"])
        git(path, "checkout", "-q", "FETCH_HEAD")
        git(path, "submodule", "update", "-q", "--init", "--recursive", "--depth", "1")


if __name__ == "__main__":
    main()
