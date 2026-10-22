"""Makes a Nav2 parameter file from a base one by dotted-path assignments, keeping every value's YAML type (a ROS 2 double parameter refuses an integer) and refusing keys that do not exist.
usage: params_variant.py <base.yaml> <out.yaml> <path=value> ...      e.g. amcl.ros__parameters.alpha1=0.05  controller_server.ros__parameters.FollowPath.desired_linear_vel=0.4
Values are YAML scalars or flow lists; a list replaces the whole list. Comments of the base file are not kept (the output is for experiments, the committed config is edited by hand)."""
import copy
import sys

import yaml


def _coerce(old, new, path):
    if isinstance(old, bool) or isinstance(new, bool):
        if isinstance(old, bool) and isinstance(new, bool):
            return new
    elif isinstance(old, float) and isinstance(new, (int, float)):
        return float(new)
    elif isinstance(old, int) and isinstance(new, int):
        return new
    elif isinstance(old, str) and isinstance(new, str):
        return new
    elif isinstance(old, list) and isinstance(new, list):
        return new
    raise ValueError(f"{path}: cannot set a {type(old).__name__} parameter to {new!r} ({type(new).__name__})")


def apply(base, assignments):
    out = copy.deepcopy(base)
    for a in assignments:
        if "=" not in a:
            raise ValueError(f"{a!r}: expected path=value")
        path, raw = a.split("=", 1)
        keys, node = path.split("."), out
        for k in keys[:-1]:
            if not isinstance(node, dict) or k not in node:
                raise ValueError(f"{path}: no such key {k!r}")
            node = node[k]
        if not isinstance(node, dict) or keys[-1] not in node:
            raise ValueError(f"{path}: no such parameter {keys[-1]!r}")
        node[keys[-1]] = _coerce(node[keys[-1]], yaml.safe_load(raw), path)
    return out


def main(argv):
    if len(argv) < 3:
        raise SystemExit(__doc__)
    out = apply(yaml.safe_load(open(argv[0])), argv[2:])
    yaml.safe_dump(out, open(argv[1], "w"), default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    main(sys.argv[1:])
