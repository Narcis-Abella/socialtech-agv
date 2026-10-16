"""report.json of a make_map.sh run: what the mapping run and the map say about themselves, so a bad map shows without opening it.
usage: map_report.py <out_dir>   (reads vs/run/alidarState.txt, node.log, ply_to_map.log, map.yaml, map.pgm; writes <out_dir>/report.json and prints it)"""
import json
import re
import sys
from pathlib import Path

import numpy as np


def run_stats(out):
    """Voxel-SLAM side: resets, poses, path length and start-to-end distance (our bags end where they start, so that distance is the loop-closure error)."""
    p = np.loadtxt(out / "vs/run/alidarState.txt", usecols=range(1, 4), ndmin=2)
    return {"resets": (out / "node.log").read_text(errors="replace").count("resetting system"), "poses": len(p),
            "path_m": round(float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()), 2),
            "end_to_start_m": round(float(np.linalg.norm(p[-1] - p[0])), 2)}


def floor_stats(log):
    """From ply_to_map's summary line: the floor tilt and the sensor height its trajectory vote found."""
    s = Path(log).read_text(errors="replace")
    tilt, h = re.search(r"floor tilt ([0-9.]+) deg", s), re.search(r"'height': ([0-9.]+)", s)
    return {"floor_tilt_deg": float(tilt.group(1)) if tilt else None, "sensor_height_m": round(float(h.group(1)), 3) if h else None}


def map_stats(prefix):
    """Size, resolution and cell counts of a map_server PGM + YAML (trinary: occupied 0, free 254, unknown 205)."""
    yml = Path(str(prefix) + ".yaml").read_text()
    raw = Path(str(prefix) + ".pgm").read_bytes()
    head = re.match(rb"P5\n(\d+) (\d+)\n255\n", raw)
    g = np.frombuffer(raw[head.end():], np.uint8)
    return {"resolution_m": float(re.search(r"^resolution: ([0-9.]+)", yml, re.M).group(1)), "size_cells": [int(head.group(1)), int(head.group(2))],
            "occupied": int((g == 0).sum()), "free": int((g == 254).sum()), "unknown": int((g == 205).sum()), "has_T_map_world": "# T_map_world" in yml}


def main(out):
    out = Path(out)
    rep = {**run_stats(out), **floor_stats(out / "ply_to_map.log"), "map": map_stats(out / "map")}
    (out / "report.json").write_text(json.dumps(rep, indent=1) + "\n")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
