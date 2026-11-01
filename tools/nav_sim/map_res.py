"""Same map as slam_bench/map_visibility.py (ply_to_map levelling + ghost filter + free-space fill) at another resolution.

  map_res.py <bag> <poses.txt> <world.ply> <out_prefix> <res_m> [--topic /livox/lidar]

map_visibility.main fixes 0.05 m although build_map takes `res`. --min-component is scaled by the cell area (10 cells at 5 cm = 40 at 2.5 cm: same physical speck size);
min_hits (2 points per cell) and the 0.3 m fill are left as they are. Run inside the mapper image (needs rosbags)."""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "slam_bench"))
import map_visibility as mv   # noqa: E402
import ply_to_map as pm       # noqa: E402


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("poses")
    ap.add_argument("world_ply")
    ap.add_argument("out_prefix")
    ap.add_argument("res", type=float)
    ap.add_argument("--topic", default="/livox/lidar")
    a = ap.parse_args(argv)
    P = pm.read_ply(a.world_ply)
    Q, info = pm.level_to_floor(P)
    poses = np.loadtxt(a.poses, usecols=range(8))
    min_comp = round(mv.MIN_COMPONENT * (0.05 / a.res) ** 2)
    grid, origin, _ = mv.build_map(P, Q, info["T"], lambda: mv.bag_scans(a.bag, poses, a.topic), poses[:, 1:4], res=a.res, min_component=min_comp)
    pm.write_map(a.out_prefix, grid, a.res, origin, info["T"])
    print(f"res {a.res}: {grid.shape[1]}x{grid.shape[0]} cells: {(grid == pm.OCC).mean():.1%} occupied, {(grid == pm.FREE).mean():.1%} free, {(grid == pm.UNK).mean():.1%} unknown")


if __name__ == "__main__":
    main(sys.argv[1:])
