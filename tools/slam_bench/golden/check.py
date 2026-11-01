"""Does the new make_map reproduce the pipeline it replaced? Runs make_map.build with the ghost test limited to the old band on the frozen inputs and compares sha256 with r03.sha256:
map.pgm, map.yaml and the old `map_clean.ply` (the band points over the occupied cells of the final grid, rebuilt here from the new whole-height map_clean.ply).
  python3 check.py BAG WORLD_PLY ALIDARSTATE OUT_DIR       (inside the mapper image, tools/slam_bench on the path; see README.md)"""
import hashlib
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cloud_io  # noqa: E402
import make_map  # noqa: E402

BAND = (0.10, 1.80)
RES = 0.025


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def expected():
    """{file name: sha256} from r03.sha256 (the lines `<hash>  <name>  [comment]`)."""
    return {m.group(2): m.group(1) for m in (re.match(r"^([0-9a-f]{64})  (\S+)", l) for l in (Path(__file__).parent / "r03.sha256").read_text().splitlines()) if m}


def old_map_clean(clean, pgm, yaml, res):
    """The cloud the old map showed: points of the obstacle band over occupied cells (row 0 = max y), in the order of the cleaned cloud."""
    raw = Path(pgm).read_bytes()
    w, h = map(int, raw.split(b"\n")[1].split())
    grid = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)
    ox, oy = (float(v) for v in re.search(r"^origin: \[([^\]]+)\]", Path(yaml).read_text(), re.M).group(1).split(",")[:2])
    Q = clean[(clean[:, 2] > BAND[0]) & (clean[:, 2] <= BAND[1])]
    c = np.floor((Q[:, 0] - ox) / res).astype(int)
    r = h - 1 - np.floor((Q[:, 1] - oy) / res).astype(int)
    ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
    keep = np.zeros(len(Q), bool)
    keep[ok] = grid[r[ok], c[ok]] == 0
    return Q[keep]


def main(bag, world, poses_file, out):
    out = Path(out)
    poses = np.loadtxt(poses_file, usecols=range(8), ndmin=2)
    make_map.build(cloud_io.read_ply(world), poses, bag, out, res=RES, ceil=1.80, ghost_band=BAND)
    cloud_io.write_ply(out / "map_clean_old.ply", old_map_clean(cloud_io.read_ply(out / "map_clean.ply"), out / "map.pgm", out / "map.yaml", RES))
    want, bad = expected(), []
    for got, name in (("map.pgm", "map.pgm"), ("map.yaml", "map.yaml"), ("map_clean_old.ply", "map_clean.ply")):
        ok = sha(out / got) == want[name]
        print(f"{'OK  ' if ok else 'DIFF'} {name}  {sha(out / got)}")
        bad += [] if ok else [name]
    print("GOLDEN OK" if not bad else f"GOLDEN FAILED: {', '.join(bad)}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:5]))
