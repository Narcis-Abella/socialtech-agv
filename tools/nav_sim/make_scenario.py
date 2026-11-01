"""Writes a start/goal scenario file from a map. usage: make_scenario.py <map.yaml> <out.json> <n> <seed> [clearance=0.45] [dmin=4] [dmax=20] [goal_band]
The pairs are independent trials (the robot is put at `start` before each), with the reference shortest length for SPL."""
import json
import sys

import scenarios as sc


def main(map_yaml, out, n, seed, clearance=0.45, dmin=4.0, dmax=20.0, goal_band=None):
    mp = sc.load_map(map_yaml)
    pairs = sc.sample_pairs(mp["blocked"], int(n), int(seed), float(clearance), float(dmin), float(dmax), mp["res"], mp["origin"], float(goal_band) if goal_band else None)
    json.dump({"map": map_yaml.split("/")[-1], "n": int(n), "seed": int(seed), "clearance": float(clearance), "dmin": float(dmin), "dmax": float(dmax), "goal_band": float(goal_band) if goal_band else None, "trials": pairs},
              open(out, "w"), indent=1)
    print(f"{len(pairs)} pairs, shortest/euclid median {sorted(p['shortest'] / p['euclid'] for p in pairs)[len(pairs) // 2]:.2f}")


if __name__ == "__main__":
    main(*sys.argv[1:])
