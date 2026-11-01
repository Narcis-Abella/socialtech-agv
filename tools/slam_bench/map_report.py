"""report.json of a make_map run: what the mapping run and the map say about themselves, so a bad map shows without opening it. The floor tilt, the sensor height and the
provenance (pipeline marker, commit, rule constants) come from the pipeline; the Voxel-SLAM side from the run's poses and node.log; the lidar message count from the bag metadata."""
import json
import re
from pathlib import Path

import numpy as np
import yaml


def run_stats(poses_file, node_log=None):
    """Voxel-SLAM side: poses, path length, start-to-end distance (our bags end where they start, so that distance is the loop-closure error) and, when there is a node.log
    (a map regenerated from an old folder has none), the resets and the `Insufficient IMU` lines."""
    p = np.loadtxt(poses_file, usecols=range(1, 4), ndmin=2)
    log = Path(node_log).read_text(errors="replace") if node_log and Path(node_log).exists() else None
    return {"resets": None if log is None else log.count("resetting system"), "insufficient_imu": None if log is None else log.count("Insufficient IMU"), "poses": len(p),
            "path_m": round(float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()), 2),
            "end_to_start_m": round(float(np.linalg.norm(p[-1] - p[0])), 2)}


def scans_in_bag(bag, topic):
    """Message count of `topic` from the bag's metadata.yaml, None when the file or the topic is not there."""
    f = Path(bag) / "metadata.yaml"
    try:
        for t in yaml.safe_load(f.read_text())["rosbag2_bagfile_information"]["topics_with_message_count"]:
            if t["topic_metadata"]["name"] == topic:
                return int(t["message_count"])
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError):     # the lost-scan check is optional: another layout of the file must not cost the map built before it
        pass
    return None


def map_stats(prefix):
    """Size, resolution and cell counts of a map_server PGM + YAML (trinary: occupied 0, free 254, unknown 205)."""
    yml = Path(str(prefix) + ".yaml").read_text()
    raw = Path(str(prefix) + ".pgm").read_bytes()
    head = re.match(rb"P5\n(\d+) (\d+)\n255\n", raw)
    g = np.frombuffer(raw[head.end():], np.uint8)
    return {"resolution_m": float(re.search(r"^resolution: ([0-9.]+)", yml, re.M).group(1)), "size_cells": [int(head.group(1)), int(head.group(2))],
            "occupied": int((g == 0).sum()), "free": int((g == 254).sum()), "unknown": int((g == 205).sum()), "has_T_map_world": "# T_map_world" in yml}


def write_report(out, *, poses_file, node_log, extra):
    """Write <out>/report.json (Voxel-SLAM side + extra + the map files of <out>) and return it."""
    out = Path(out)
    rep = {**run_stats(poses_file, node_log), **extra, "map": map_stats(out / "map")}
    (out / "report.json").write_text(json.dumps(rep, indent=1) + "\n")
    return rep
