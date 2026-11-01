"""Checks of merge_params.merge (no ROS needed): python3 ops/test_merge_params.py"""
import copy
import os
import subprocess
import sys
import tempfile

from merge_params import merge

base = {"a": {"x": 1, "y": {"z": 2}}, "b": 3}
assert merge({}, base) == base
assert merge(copy.deepcopy(base), {"a": {"y": {"z": 9}}}) == {"a": {"x": 1, "y": {"z": 9}}, "b": 3}      # nested keys merge, the later file wins
assert merge(copy.deepcopy(base), None) == base                                                          # an empty / comment-only YAML loads as None
assert merge({}, {}) == {}

# the overlay is not aliased into the output: merging a second file must not edit the first one's dict
first = {"a": {"x": 1}}
out = merge({}, first)
merge(out, {"a": {"x": 2}})
assert first == {"a": {"x": 1}}, first

# end to end, as nav_live.sh calls it: a base plus an overlay trimmed back to comments only ("to go back, remove those lines")
with tempfile.TemporaryDirectory() as d:
    b, e = os.path.join(d, "base.yaml"), os.path.join(d, "empty.yaml")
    open(b, "w").write("a:\n  x: 1\n")
    open(e, "w").write("# nothing to override\n")
    r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "merge_params.py"), b, e], capture_output=True, text=True)
    assert r.returncode == 0 and "x: 1" in r.stdout, (r.returncode, r.stderr)
print("ok")
