"""2D occupancy map (map_server PGM + YAML) from a SLAM point cloud in PLY: finds and levels the floor, then drops floor and ceiling.

  ply_to_map.py <in.ply> <out_prefix> [--res 0.05] [--ceil 1.80] [--floor-band 0.10] [--min-hits 2] [--fill 0.3] [--no-align] [--min-manhattan 0.7] [--min-floor 250] [--max-below 0.10] [--traj poses.tum]

Needs numpy + scipy (host/laptop tool, like floor_tilt.py; scipy is not in the robot image). Input: x, y, z only, so it works for any SLAM.
1. Floor: 10 cm voxels, PCA normals (k=16), planar voxels with |nz| > 0.9, sequential RANSAC (up to 6 planes); the LOWEST plane is the floor
   (the biggest near-horizontal plane is usually the ceiling: the Mid-360 looks mostly up). No assumption about where the PLY origin is.
2. Refuses (non-zero exit) instead of guessing:
   - floor support < --min-floor voxels: the floor was not seen (a smaller plane would silently be the ceiling);
   - more than --max-below of the map lies > 0.3 m BELOW the plane: it is a ceiling or a table (floors measured 0.00-0.06, ceilings 0.32+);
   - floor steeper than 10 deg (floor_tilt.MAX_TILT; checked after the fit, RANSAC alone only filters its hypotheses).
   Multi-level maps (lifts, ramps) are not supported and are not always detected.
3. Optional --traj (TUM poses), two independent votes that the plane is the floor, each skipped when the trajectory cannot vote:
   position: path >= 20 m and not a straight line -> floor parallel to the trajectory plane (< 1 deg) and below it;
   orientation: >= 90 deg of turning about one clean axis -> the robot's yaw axis is the floor normal (< 1 deg), whatever the sensor mount.
   No sensor-height prior: a desk 0.60 m under the sensor would pass one.
4. The cloud is rotated so the floor is horizontal (tilt printed) and heights become metres above the floor (the PLY origin is the sensor). Then the
   map is turned about z so the walls lie on the pixel rows/columns (Manhattan: circular mean of 4*theta of the wall normals; only if the walls are
   Manhattan enough, --min-manhattan; --no-align to keep the SLAM world's yaw). The transform PLY -> map is written in the YAML as a comment.
   Heights: < floor-band = floor (marks free), floor-band..ceil = obstacle, > ceil = dropped (ceiling, door frames).
5. Cells: occupied (0) with >= min-hits obstacle points, else free (254) with floor points, else unknown (205).
Free space comes only from floor points the sensor actually saw (sparse: --fill closes gaps of about 0.3 m), so large areas the LiDAR never saw the floor of stay unknown.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_closing
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import floor_tilt
from static_offset import rot_to_z

OCC, FREE, UNK = 0, 254, 205
PLY_TYPES = {"char": "i1", "uchar": "u1", "short": "<i2", "ushort": "<u2", "int": "<i4", "uint": "<u4", "float": "<f4", "double": "<f8"}
VOXEL = 0.10        # m, downsampling before the normals
K = 16              # neighbours for a voxel's normal
PLANAR = 0.01       # smallest / total covariance eigenvalue of the neighbourhood: below this the voxel is on a plane
MAX_PLANES = 6
MIN_SEED = 50       # voxels: smallest plane considered while choosing the lowest; --min-floor is checked AFTER the choice
STRAY_PTS = 5       # points a 1 m cell needs to count for the map extent (walls and floor have hundreds)
WALL_NZ, WALL_LO, WALL_HI, MIN_WALL = 0.1, 0.2, 1.6, 50  # a wall voxel: |nz| below, metres above the floor between, at least this many voxels to estimate
BELOW = 0.3         # m: voxels this far under the chosen plane mean it is not the floor
TRAJ_MIN_LEN, TRAJ_MIN_SHAPE, TRAJ_MAX_ANGLE = 20.0, 0.2, 1.0  # position vote: m, minor/major singular value, deg
YAW_POSES, YAW_MIN_RANGE, YAW_MAX_RATIO, YAW_MAX_ANGLE = 150, 90.0, 0.01, 1.0  # orientation vote: poses used, deg turned, eigenvalue ratio, deg


def read_ply(path):
    """(N, 3) float64 xyz of a binary little-endian PLY (other properties ignored)."""
    raw = Path(path).read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    head = raw[:end].decode().splitlines()
    if "format binary_little_endian 1.0" not in head:
        sys.exit(f"error: {path} is not a binary little-endian PLY")
    n = int(next(l.split()[2] for l in head if l.startswith("element vertex")))
    props = [(l.split()[2], PLY_TYPES[l.split()[1]]) for l in head if l.startswith("property")]
    a = np.frombuffer(raw, dtype=np.dtype(props), count=n, offset=end)
    return np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float64)


def read_traj(path):
    """(positions (N, 3), quaternions xyzw (N, 4)) of a TUM trajectory (t x y z qx qy qz qw)."""
    a = np.loadtxt(path)
    return a[:, 1:4], a[:, 4:8]


def voxel_centroids(P, size):
    idx = np.floor(P / size).astype(np.int64)
    idx -= idx.min(0)
    key = (idx[:, 0] * (idx[:, 1].max() + 1) + idx[:, 1]) * (idx[:, 2].max() + 1) + idx[:, 2]
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inv.ravel(), P[:, i]) for i in range(3)]) / cnt[:, None]


def pca_normals(V):
    """Unsigned unit normal and planarity (smallest / total eigenvalue) of each voxel's K nearest neighbours."""
    nb = cKDTree(V).query(V, k=K, workers=-1)[1]
    c = V[nb] - V[nb].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum("nki,nkj->nij", c, c) / K)
    return v[:, :, 0], w[:, 0] / (w.sum(1) + 1e-12)


def yaw_axis(quat):
    """(axis, ratio, yaw_range_deg): the axis every rotation of the trajectory turns about. For a robot driving on a flat floor it is the floor normal,
    whatever the LiDAR mount (the constant mount rotation cancels in R_j R_i^T). ratio = smallest / second eigenvalue (0 = one clean axis)."""
    R = Rotation.from_quat(quat)
    sub = R[np.linspace(0, len(R) - 1, min(len(R), YAW_POSES)).astype(int)]
    i, j = np.triu_indices(len(sub), 1)
    rel = (sub[j] * sub[i].inv()).as_matrix() - np.eye(3)
    w, v = np.linalg.eigh(np.einsum("nki,nkj->ij", rel, rel))
    axis = v[:, 0]
    turn = np.unwrap((R * R[0].inv()).as_rotvec() @ axis)
    return axis, float(w[0] / w[1]) if w[1] > 1e-12 else float("nan"), float(np.degrees(np.ptp(turn)))


def check_trajectory(traj, v, d):
    """Two votes that the plane (v, d) is the floor: {"position": ..., "orientation": ...}, each "skipped" when the trajectory cannot vote, else a dict.
    position: the path (>= 20 m, not a straight line) lies on a plane parallel to the floor, above it.
    orientation: the robot's yaw axis (>= 90 deg turned, one clean axis) is the floor normal. ValueError if a voter that can vote disagrees."""
    positions, quat = traj
    out = {"position": "skipped", "orientation": "skipped"}
    length = np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
    s, vt = np.linalg.svd(positions - positions.mean(0), full_matrices=False)[1:]
    if length >= TRAJ_MIN_LEN and s[1] / s[0] >= TRAJ_MIN_SHAPE:
        angle = float(np.degrees(np.arccos(min(1.0, abs(vt[2] @ v)))))
        height = float(np.median(positions @ v - d))  # how far the robot drove above the plane
        if angle > TRAJ_MAX_ANGLE or height <= 0:
            raise ValueError(f"trajectory check failed (position plane): the plane is {angle:.2f} deg from the trajectory plane and {height:.2f} m below it "
                             f"(need < {TRAJ_MAX_ANGLE} deg and below the robot): not the floor")
        out["position"] = {"angle_deg": angle, "height": height}
    axis, ratio, yaw = yaw_axis(quat)
    if yaw >= YAW_MIN_RANGE and ratio < YAW_MAX_RATIO:  # nan (no rotation at all) fails the comparison: skipped
        angle = float(np.degrees(np.arccos(min(1.0, abs(axis @ v)))))
        if angle > YAW_MAX_ANGLE:
            raise ValueError(f"trajectory check failed (yaw axis): the robot turned about an axis {angle:.2f} deg from the plane normal (need < {YAW_MAX_ANGLE} deg): not the floor")
        out["orientation"] = {"angle_deg": angle, "yaw_deg": yaw}
    return out


def manhattan_yaw(n):
    """(yaw_deg, strength) from horizontal wall normals n (N, 2): turning the cloud by yaw_deg (in (-45, 45]) puts the walls on the axes.
    Circular mean of 4*theta: the factor 4 folds the four wall directions of a rectangular building onto one. strength (0-1) is how Manhattan the
    walls are: 1 = all at multiples of 90 deg, ~0 = every angle."""
    z = np.exp(4j * np.arctan2(n[:, 1], n[:, 0])).mean()
    return float(np.degrees(-np.angle(z) / 4)), float(abs(z))


def level_to_floor(P, min_floor=250, max_below=0.10, traj=None, align=True, min_manhattan=0.7):
    """(Q, info): P rotated so the floor is horizontal and shifted so the floor is z = 0, then (align, if the walls are Manhattan enough) turned about z
    so the walls lie on the x/y axes. info: tilt_deg, resid_std (m), n_floor, below, traj, yaw_deg (turn applied), manhattan (wall strength),
    T (4x4, input frame -> output frame)."""
    V = voxel_centroids(P, VOXEL)
    nrm, planar = pca_normals(V)
    cand = V[(np.abs(nrm[:, 2]) > 0.9) & (planar < PLANAR)]
    rng = np.random.default_rng(0)
    planes = []
    for _ in range(MAX_PLANES):
        plane = floor_tilt.fit_plane(cand, rng, MIN_SEED)
        if plane is None:
            break
        planes.append(plane)
        cand = cand[np.abs(cand @ plane[0] - plane[1]) >= floor_tilt.TOL]
    if not planes:
        raise ValueError("no floor plane found (nothing flat and near-horizontal within 10 deg)")
    c = V.mean(0)
    v, d, n = min(planes, key=lambda p: (p[1] - p[0][0] * c[0] - p[0][1] * c[1]) / p[0][2])  # lowest at the map centre
    if n < min_floor:
        raise ValueError(f"floor support is {n} voxels, below --min-floor {min_floor}: the floor was probably not seen (a smaller plane would be the ceiling)")
    tilt = float(np.degrees(np.arccos(np.clip(v[2], -1, 1))))
    if tilt > floor_tilt.MAX_TILT:  # fit_plane only limits its RANSAC hypotheses: the least-squares refit can end steeper
        raise ValueError(f"floor tilt {tilt:.1f} deg exceeds {floor_tilt.MAX_TILT}: the SLAM start or the plane is wrong, not the floor")
    dist = V @ v - d
    below = float(np.mean(dist < -BELOW))
    if below > max_below:
        raise ValueError(f"{below:.0%} of the map lies more than {BELOW} m below the chosen plane (floors: < 6%): it is a ceiling or a table, not the floor")
    checked = "not given" if traj is None else check_trajectory(traj, v, d)
    Rl = rot_to_z(v)
    nl, hv = nrm @ Rl.T, V @ Rl[2] - d  # voxel normals and heights above the floor in the levelled frame
    wall = (np.abs(nl[:, 2]) < WALL_NZ) & (planar < PLANAR) & (hv > WALL_LO) & (hv < WALL_HI)
    yaw, strength = manhattan_yaw(nl[wall, :2]) if wall.sum() >= MIN_WALL else (0.0, 0.0)
    turn = yaw if align and strength >= min_manhattan else 0.0
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", turn, degrees=True).as_matrix() @ Rl
    T[2, 3] = -d
    return P @ T[:3, :3].T + T[:3, 3], {"tilt_deg": tilt, "resid_std": float(dist[np.abs(dist) < floor_tilt.TOL].std()), "n_floor": n, "below": below,
                                        "traj": checked, "yaw_deg": turn, "manhattan": strength, "T": T}


def rasterize(Q, res, ceil=1.80, floor_band=0.10, min_hits=2, fill=0.30):
    """(grid, origin): grid[row, col] in OCC/FREE/UNK, row 0 = max y (PGM order); origin = (x, y) of the lower-left corner.
    Only floor and obstacle points count; points alone in their 1 m cell (fewer than STRAY_PTS) are dropped: stray points far away would inflate the grid.
    fill (m): free space is closed over gaps up to about this size (the floor is seen sparsely); obstacles are never overwritten, 0 = off."""
    h = Q[:, 2]
    P = Q[(h > -floor_band) & (h <= ceil)]
    ij = np.floor(P[:, :2]).astype(np.int64)
    ij -= ij.min(0)
    _, inv, cnt = np.unique(ij[:, 0] * (ij[:, 1].max() + 1) + ij[:, 1], return_inverse=True, return_counts=True)
    P = P[cnt[inv.ravel()] >= STRAY_PTS]
    origin = (np.floor(P[:, 0].min() / res) * res, np.floor(P[:, 1].min() / res) * res)
    cols = ((P[:, 0] - origin[0]) / res).astype(int)
    rows = ((P[:, 1] - origin[1]) / res).astype(int)
    shape = (rows.max() + 1, cols.max() + 1)

    def count(sel):
        return np.bincount(rows[sel] * shape[1] + cols[sel], minlength=shape[0] * shape[1]).reshape(shape)

    obstacle = count(P[:, 2] >= floor_band)
    floor = count(np.abs(P[:, 2]) < floor_band)
    grid = np.full(shape, UNK, dtype=np.uint8)
    grid[floor > 0] = FREE
    if fill > 0:
        r = max(1, round(fill / res))
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        closed = binary_closing(grid == FREE, structure=(x * x + y * y <= r * r))
        grid[closed & (grid == UNK)] = FREE
    grid[obstacle >= min_hits] = OCC
    return grid[::-1], origin


def write_map(prefix, grid, res, origin, T=None):
    """PGM + map_server YAML. T (4x4, the PLY's frame -> the map's frame) goes in as a comment: map_server ignores it, you need it to relate waypoints to the SLAM world."""
    prefix = Path(prefix)
    prefix.with_suffix(".pgm").write_bytes(b"P5\n%d %d\n255\n" % (grid.shape[1], grid.shape[0]) + grid.tobytes())
    # free_thresh 0.196, not Nav2's 0.25: map_server reads a pixel as free when (255 - pixel) / 255 <= free_thresh, and unknown (205) is 0.19608
    prefix.with_suffix(".yaml").write_text(f"image: {prefix.name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n"
                                           "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
                                           + ("" if T is None else "# T_map_world (row-major; p_map = T p_ply):\n" + "".join(f"# {row}\n" for row in np.asarray(T).tolist())))


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ply")
    ap.add_argument("out_prefix")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--ceil", type=float, default=1.80, help="m above the floor; points higher are dropped")
    ap.add_argument("--floor-band", type=float, default=0.10, help="m above the floor still counted as floor")
    ap.add_argument("--min-hits", type=int, default=2)
    ap.add_argument("--fill", type=float, default=0.30, help="m: close gaps in the free space up to about this size (0 = off)")
    ap.add_argument("--min-floor", type=int, default=250, help="voxels (10 cm) the floor plane must have; smallest real floors seen: 193-300")
    ap.add_argument("--max-below", type=float, default=0.10, help="share of the map allowed > 0.3 m below the floor plane")
    ap.add_argument("--no-align", action="store_true", help="keep the SLAM world's yaw (by default the map is turned so the walls lie on the pixel rows/columns)")
    ap.add_argument("--min-manhattan", type=float, default=0.7, help="minimum wall strength (0-1) to turn the map; round or diagonal buildings stay as they are. Measured: our 9 maps 0.89-0.99, other datasets 0.33-0.57")
    ap.add_argument("--traj", help="TUM trajectory: cross-check that the floor is parallel to it (skipped if it is short or straight)")
    a = ap.parse_args(argv)
    try:
        Q, info = level_to_floor(read_ply(a.ply), a.min_floor, a.max_below, None if a.traj is None else read_traj(a.traj), not a.no_align, a.min_manhattan)
    except ValueError as e:
        sys.exit(f"error: {e}")
    grid, origin = rasterize(Q, a.res, a.ceil, a.floor_band, a.min_hits, a.fill)
    write_map(a.out_prefix, grid, a.res, origin, info["T"])
    n = len(Q)
    print(f"floor tilt {info['tilt_deg']:.2f} deg (levelled), {info['n_floor']} floor voxels, residual {info['resid_std'] * 100:.1f} cm, "
          f"{info['below']:.1%} of the map below it, trajectory check: {info['traj']}; map turned {info['yaw_deg']:+.1f} deg (wall strength {info['manhattan']:.2f}); "
          f"{(np.abs(Q[:, 2]) < a.floor_band).sum() / n:.1%} floor, {((Q[:, 2] >= a.floor_band) & (Q[:, 2] <= a.ceil)).sum() / n:.1%} obstacle, {(Q[:, 2] > a.ceil).sum() / n:.1%} above {a.ceil} m; "
          f"{grid.shape[1]}x{grid.shape[0]} cells: {(grid == OCC).mean():.1%} occupied, {(grid == FREE).mean():.1%} free, {(grid == UNK).mean():.1%} unknown")


if __name__ == "__main__":
    main(sys.argv[1:])
