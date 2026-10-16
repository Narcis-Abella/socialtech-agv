"""Checks map_report.py on a synthetic run directory. Run: python3 test_map_report.py"""
import json
import tempfile
from pathlib import Path

import map_report

# the one-line summary ply_to_map prints (copied from a real run on a Voxel-SLAM map)
PLY_LOG = ("floor tilt 0.94 deg (levelled), 9259 floor voxels, residual 1.7 cm, 0.1% of the map below it, trajectory check: "
           "{'position': {'angle_deg': 0.088, 'height': 1.3924993401931398}, 'orientation': 'skipped'}; map turned -18.5 deg (wall strength 0.93); "
           "2.2% floor, 97.1% obstacle, 0.1% above 30.0 m; 7093x5139 cells: 2.6% occupied, 1.6% free, 95.8% unknown\n")


def make_run(tmp, poses, node_log="", pgm=b"\x00\xfe\xcd\xcd", yaml_extra="# T_map_world (row-major; p_map = T p_ply):\n"):
    out = Path(tmp)
    (out / "vs/run").mkdir(parents=True)
    (out / "vs/run/alidarState.txt").write_text("".join(f"{100 + i} {x} {y} {z} 0 0 0 1\n" for i, (x, y, z) in enumerate(poses)))
    (out / "node.log").write_text(node_log)
    (out / "ply_to_map.log").write_text(PLY_LOG)
    (out / "map.yaml").write_text("image: map.pgm\nmode: trinary\nresolution: 0.05\n" + yaml_extra)
    (out / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + pgm)
    return out


def test_report_counts_resets_poses_path_closure_height_and_cells():
    with tempfile.TemporaryDirectory() as tmp:
        out = make_run(tmp, [(0, 0, 0), (3, 0, 0), (3, 4, 0)], node_log="a\nOdometry degradation detected - resetting system\nb\nresetting system\n")
        map_report.main(out)
        r = json.loads((out / "report.json").read_text())
        assert (r["resets"], r["poses"], r["path_m"], r["end_to_start_m"]) == (2, 3, 7.0, 5.0)
        assert (r["floor_tilt_deg"], r["sensor_height_m"]) == (0.94, 1.392)
        assert r["map"] == {"resolution_m": 0.05, "size_cells": [2, 2], "occupied": 1, "free": 1, "unknown": 2, "has_T_map_world": True}


def test_a_run_with_one_pose_and_a_map_without_occupied_cells():
    with tempfile.TemporaryDirectory() as tmp:
        out = make_run(tmp, [(1, 2, 3)], pgm=b"\xfe\xfe\xcd\xcd", yaml_extra="")
        map_report.main(out)
        r = json.loads((out / "report.json").read_text())
        assert (r["poses"], r["path_m"], r["end_to_start_m"], r["resets"]) == (1, 0.0, 0.0, 0)
        assert r["map"]["occupied"] == 0 and r["map"]["has_T_map_world"] is False


if __name__ == "__main__":
    test_report_counts_resets_poses_path_closure_height_and_cells()
    test_a_run_with_one_pose_and_a_map_without_occupied_cells()
    print("ok")
