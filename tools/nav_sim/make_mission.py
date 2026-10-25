"""Turns a start/goal scenario into multi-stop missions for trials.py (NavigateThroughPoses). usage: make_mission.py <scenario.json> <out.json> <stops>
Mission k takes `stops` consecutive pairs: it starts at the first pair's start, passes through the goals of all but the last pair and ends at the last pair's goal.
`shortest` is the straight-line chain through the stops (a lower bound, not a path length): SPL on a mission is only a relative efficiency."""
import json
import math
import sys


def build(trials, stops):
    out = []
    for k in range(len(trials) // stops):
        g = trials[k * stops:(k + 1) * stops]
        pts = [g[0]["start"]] + [t["goal"] for t in g]
        out.append({"id": f"m{k:02d}", "start": g[0]["start"], "via": [t["goal"] for t in g[:-1]], "goal": g[-1]["goal"],
                    "shortest": sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))})
    return out


def main(scenario, out, stops):
    d = json.load(open(scenario))
    d["trials"] = build(d["trials"], int(stops))
    json.dump(d, open(out, "w"), indent=1)
    print(f"{len(d['trials'])} missions of {stops} stops")


if __name__ == "__main__":
    main(*sys.argv[1:])
