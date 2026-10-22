"""Evaluator-only: express session-B reference poses in session-A's Voxel-SLAM world frame.

p_worldA = inv(T_mapA_worldA) @ T_A_B(link) @ T_mapB_worldB @ p_B, so that bench
scoring against map A (pose_eval.py report) works for a session-B bag.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from offline_s2 import map_transform


def convert(rows, T):
    out = rows.copy()
    out[:, 1:4] = rows[:, 1:4] @ T[:3, :3].T + T[:3, 3]
    out[:, 4:8] = (Rotation.from_matrix(T[:3, :3]) * Rotation.from_quat(rows[:, 4:8])).as_quat()
    return out


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--reference-b', required=True)
    p.add_argument('--yaml-b', required=True)
    p.add_argument('--yaml-a', required=True)
    p.add_argument('--link', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    T = np.linalg.inv(map_transform(a.yaml_a)) @ np.asarray(json.loads(Path(a.link).read_text())['T_A_B']) @ map_transform(a.yaml_b)
    np.savetxt(a.out, convert(np.loadtxt(a.reference_b, usecols=range(8)), T), fmt='%.9f')
