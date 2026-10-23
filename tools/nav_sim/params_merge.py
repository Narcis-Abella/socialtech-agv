"""Deep-merges an overlay YAML into a Nav2 parameter file (new keys and plugin blocks are allowed, unlike params_variant.py). Dicts are merged, lists and scalars replaced; changing the type
of an existing scalar (a ROS 2 parameter keeps its type) is refused, an int given for a float is widened.
usage: params_merge.py <base.yaml> <overlay.yaml> <out.yaml>"""
import copy
import sys

import yaml


def merge(base, over, path=""):
    out = copy.deepcopy(base)
    for k, v in over.items():
        here = f"{path}.{k}" if path else k
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = merge(out[k], v, here)
        elif k in out and not isinstance(out[k], (dict, list)):
            old = out[k]
            if isinstance(old, bool) != isinstance(v, bool) or isinstance(v, (dict, list)) or (isinstance(old, str) != isinstance(v, str)):
                raise ValueError(f"{here}: cannot change a {type(old).__name__} parameter to {v!r}")
            out[k] = float(v) if isinstance(old, float) and isinstance(v, (int, float)) and not isinstance(v, bool) else v
        else:
            out[k] = copy.deepcopy(v)
    return out


def main(argv):
    if len(argv) != 3:
        raise SystemExit(__doc__)
    out = merge(yaml.safe_load(open(argv[0])), yaml.safe_load(open(argv[1])))
    yaml.safe_dump(out, open(argv[2], "w"), default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    main(sys.argv[1:])
