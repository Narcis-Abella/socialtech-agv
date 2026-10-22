"""key=value pairs -> a ROS 2 parameter YAML (all nodes) for the launch file's overrides: overlay.py alpha1=0.05 max_beams=120 > extra.yaml"""
import json
import sys


def value(v):
    for cast in (int, float):
        try:
            return cast(v)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(v.lower(), v)


def overlay(pairs):
    out = ["/**:", "  ros__parameters:", "    use_sim_time: true"]  # never an empty block: it is a YAML null and rcl fails to parse it
    for p in pairs:
        k, v = p.split("=", 1)
        out.append(f"    {k}: {json.dumps(value(v))}")  # JSON scalars are valid YAML and keep int / float / bool / string apart
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    sys.stdout.write(overlay(sys.argv[1:]))
