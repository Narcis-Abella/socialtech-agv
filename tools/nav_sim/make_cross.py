"""Crossing-pedestrian scenario timed for a given robot speed: the walker is released where the robot must be so that, `delta` seconds after the walker would reach the robot's lane centre, the robot reaches the crossing point.
  make_cross.py <robot_v m/s> <out.json> [deltas, default -1.2..1.2 step 0.4]
delta = 0: both arrive together (contact unless one reacts); delta < 0: the robot gets there first; delta > 0: the walker first. Assumes the robot drives the straight lane at its target speed up to the crossing (checked on cross_n0 at 0.6 m/s: 0.6 m/s between x = 5 and 8)."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def trigger_x(cross_x, walker_s, walker_v, robot_v, delta):
    """x on the lane where the walker is released: the robot needs walker_s / walker_v + delta seconds to cover the rest of the way to the crossing."""
    return cross_x - robot_v * (walker_s / walker_v + delta)


def build(cross_x, half, walker_v, robot_v, deltas, start, goal, radius=0.3):
    out = []
    for d in deltas:
        x = trigger_x(cross_x, half, walker_v, robot_v, d)
        for side, a, b in (("n", half, -half), ("s", -half, half)):
            out.append({"id": f"cross_{side}_d{d:+.1f}", "start": start, "goal": goal, "euclid": 20.0, "shortest": 20.0, "reachable": True,
                        "dyn": [{"model": "b_walker", "path": [[cross_x, a], [cross_x, b]], "speed": walker_v, "trigger": [round(x + radius, 3), 0.0, radius], "size": [0.5, 0.5, 1.7], "parked": [12.0, -30.0]}]})
    return out


def main(argv):
    robot_v, out = float(argv[0]), argv[1]
    deltas = [float(v) for v in argv[2:]] or [-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2]
    scen = json.load(open(os.path.join(HERE, "scenarios", "dyn_hall.json")))
    scen["trials"] = build(11.0, 5.0, 1.0, robot_v, deltas, [2.0, 0.0, 0.0], [22.0, 0.0, 0.0])
    scen["n"] = len(scen["trials"])
    json.dump(scen, open(out, "w"), indent=1)
    print(f"{len(scen['trials'])} trials, triggers x =", sorted({t["dyn"][0]["trigger"][0] - 0.3 for t in scen["trials"]}))


if __name__ == "__main__":
    main(sys.argv[1:])
