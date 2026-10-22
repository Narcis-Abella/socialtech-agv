"""Evaluate SLAM trajectories against ground truth and print one markdown table.

Layout: <root>/<method>/<sequence>/*.tum   (one TUM file per run; GLIM: copy traj_lidar.txt as runN.tum)
Ground truth per sequence, see SEQUENCES below. Two kinds:
  mocap  -> APE RMSE after SE(3) Umeyama alignment (evo), at time offset 0 and at the best offset in OFFSETS
  aruco  -> translation error of the start->end relative pose at the GT time (M3DGR Corridor01 / Elevator01)
usage: eval_all.py <root> <gt_dir>
"""
import statistics
import sys
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.core.metrics import PoseRelation, StatisticsType
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

OFFSETS = [-0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
MAX_DIFF = 0.03
SEQUENCES = {  # name -> (kind, gt file)
    "office1": ("mocap", "office1_gt.tum"),
    "office2": ("mocap", "office2_gt.tum"),
    "Dynamic01": ("mocap", "Dynamic01.txt"),
    "Sha-turn01": ("mocap", "Sha-turn01.txt"),
    "Corridor01": ("aruco", "GTCorridor01.txt"),
    "Elevator01": ("aruco", "GTElevator01.txt"),
}


def ape(gt, est, offset):
    try:
        ref, e = sync.associate_trajectories(gt, est, MAX_DIFF, offset_2=offset)
    except Exception:  # no matching timestamps
        return None
    e.align(ref)
    m = metrics.APE(PoseRelation.translation_part)
    m.process_data((ref, e))
    return m.get_statistic(StatisticsType.rmse)


def mocap(gt_file, est_file):
    gt = file_interface.read_tum_trajectory_file(gt_file)
    est = file_interface.read_tum_trajectory_file(est_file)
    vals = {o: ape(gt, est, o) for o in OFFSETS}
    vals = {o: v for o, v in vals.items() if v is not None}
    if not vals:
        return None
    best = min(vals, key=vals.get)
    return vals.get(0.0), vals[best], best


def aruco(gt_file, est_file):
    lines = [x for x in Path(gt_file).read_text().splitlines() if x.strip()]  # blank-line count differs between files
    t_gt = np.array([float(lines[i]) for i in (3, 4, 5)])
    gt_time = float(lines[-1].split(":")[1].replace("s", ""))
    e = np.loadtxt(est_file)
    row = e[np.searchsorted(e[:, 0], e[0, 0] + gt_time, side="right") - 1]
    start = e[0]
    rel = np.linalg.inv(_mat(start)) @ _mat(row)
    return float(np.linalg.norm(t_gt - rel[:3, 3]))


def _mat(row):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
    m[:3, 3] = row[1:4]
    return m


def main(root, gt_dir):
    root, gt_dir = Path(root), Path(gt_dir)
    print("| Method | Sequence | Runs | Result (median; min-max) |\n|---|---|---|---|")
    for method in sorted(p.name for p in root.iterdir() if p.is_dir()):
        for seq, (kind, gt) in SEQUENCES.items():
            runs = sorted((root / method / seq).glob("*.tum")) if (root / method / seq).is_dir() else []
            if not runs:
                continue
            if kind == "mocap":
                res = [mocap(gt_dir / gt, r) for r in runs]
                res = [x for x in res if x]
                if not res:
                    print(f"| {method} | {seq} | {len(runs)} | no matching timestamps |")
                    continue
                z = [x[0] for x in res if x[0] is not None]
                b = [x[1] for x in res]
                off = statistics.median(x[2] for x in res)
                print(f"| {method} | {seq} | {len(res)} | APE@0: {statistics.median(z):.4f} m ({min(z):.4f}-{max(z):.4f}); best {statistics.median(b):.4f} m at {off:+.2f} s |")
            else:
                res = [aruco(gt_dir / gt, r) for r in runs]
                print(f"| {method} | {seq} | {len(res)} | closure: {statistics.median(res):.2f} m ({min(res):.2f}-{max(res):.2f}) |")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
