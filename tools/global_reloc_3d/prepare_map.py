"""Prepare the relocalizer prior (map.xyz) from the map_clean.ply of a make_map folder: the ghost-free cloud of the whole height, in the map frame (floor z = 0).
  prepare_map.py --map-dir DIR --out OUT [--voxel 0.2]
Crops z to [Z_LO, Z_HI] and thins to one point per voxel. Refuses a folder that make_map did not make with the current pipeline (report.json `pipeline` != 2): an old
map_clean.ply is only the obstacle band over occupied cells, and a prior built from it would have no floor and no ceiling without any error."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from offline_s2 import read_ply, voxel, write_xyz

Z_LO, Z_HI = -0.5, 5.0   # m above the floor: the floor and the ceiling of the rooms stay, outliers go


def pipeline_marker(map_dir):
    """`pipeline` of <map_dir>/report.json, None when the file or the field is missing."""
    f = Path(map_dir) / 'report.json'
    return json.loads(f.read_text()).get('pipeline') if f.exists() else None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--map-dir', required=True, help='folder made by make_map.sh (map_clean.ply and report.json)')
    p.add_argument('--out', required=True)
    p.add_argument('--voxel', type=float, default=.20)
    a = p.parse_args(argv)
    d = Path(a.map_dir)
    if pipeline_marker(d) != 2:
        sys.exit(f'{d}: report.json has no "pipeline": 2, so it was not made by the current make_map.sh and its map_clean.ply is not the whole-height clean cloud. '
                 'Regenerate it: make_map.sh --reuse-slam')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    raw = read_ply(d / 'map_clean.ply')
    keep = np.isfinite(raw).all(1) & (raw[:, 2] >= Z_LO) & (raw[:, 2] <= Z_HI)
    prior = voxel(raw[keep], a.voxel)
    write_xyz(out / 'map.xyz', prior)
    manifest = {'map_dir': str(d), 'pipeline': 2, 'raw_points': len(raw), 'finite_cropped_points': int(keep.sum()), 'voxel_points': len(prior), 'voxel_m': a.voxel,
                'frame': 'map', 'truth_used': False}
    (out / 'map_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
