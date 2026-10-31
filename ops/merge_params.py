"""Deep-merge ROS 2 parameter YAMLs, later files winning: merge_params.py base.yaml over.yaml ... > merged.yaml (PyYAML is in the ROS image)."""
import sys

import yaml


def merge(a, b):
    for k, v in b.items():
        a[k] = merge(a[k], v) if isinstance(v, dict) and isinstance(a.get(k), dict) else v
    return a


if __name__ == "__main__":
    out = {}
    for path in sys.argv[1:]:
        with open(path) as f:
            merge(out, yaml.safe_load(f))
    yaml.safe_dump(out, sys.stdout, sort_keys=False)
