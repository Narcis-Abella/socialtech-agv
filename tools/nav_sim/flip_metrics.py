"""Does the global planner oscillate between two routes? From the plans a trial recorded (trials.py "plan_all": [[t, [[x, y], ...]], ...]) classify each plan by the region it crosses and count route changes.
usage: flip_metrics.py <run dir> [<run dir> ...]   regions of the `flip` world: door x 11.5..12.5, |y| < 1 (the short route) and gap x 11.5..12.5, y 4.5..7 (the long one).
A flip is a change between two consecutive plans that cross a region (plans crossing neither are skipped). flips = 0 means the route chosen first was kept."""
import glob
import json
import os
import sys

REGIONS = {"door": (11.5, 12.5, -1.0, 1.0), "gap": (11.5, 12.5, 4.5, 7.0)}


def classify(xy, regions):
    for name, (x0, x1, y0, y1) in regions.items():
        if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in xy):
            return name
    return "none"


def flips(seq):
    known = [s for s in seq if s != "none"]
    return sum(a != b for a, b in zip(known, known[1:]))


def trial_summary(rec, regions=REGIONS):
    seq = [classify(xy, regions) for _, xy in rec.get("plan_all", [])]
    known = [s for s in seq if s != "none"]
    return {"id": rec["id"], "status": rec["status"], "duration": rec["duration"], "recoveries": rec.get("recoveries", 0), "n_plans": len(seq), "flips": flips(seq),
            "first": known[0] if known else None, "last": known[-1] if known else None}


def load_run(d):
    return [trial_summary(json.load(open(f))) for f in sorted(glob.glob(os.path.join(d, "trial_*.json")))]


def main(dirs):
    for d in dirs:
        print(f"== {d}")
        for r in load_run(d):
            print(f"{r['id']:12s} {r['status']:10s} t={r['duration']:6.1f}s plans={r['n_plans']:3d} flips={r['flips']} first={r['first']} last={r['last']} rec={r['recoveries']}")


if __name__ == "__main__":
    main(sys.argv[1:])
