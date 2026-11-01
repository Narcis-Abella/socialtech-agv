"""Find the floor of a SLAM cloud, level it (floor at z = 0), turn it so the walls lie on the axes, and cross-check the floor with the trajectory. numpy + scipy.
The cloud comes out in the map frame: heights are metres above the floor. Refuses (ValueError) instead of guessing; see level_to_floor."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import floor_tilt
from static_offset import rot_to_z

VOXEL = 0.10        # m, downsampling before the normals
K = 16              # neighbours for a voxel's normal
PLANAR = 0.01       # smallest / total covariance eigenvalue of the neighbourhood: below this the voxel is on a plane
MAX_PLANES = 6
MIN_SEED = 50       # voxels: smallest plane considered while choosing the lowest; --min-floor is checked AFTER the choice
WALL_NZ, WALL_LO, WALL_HI, MIN_WALL = 0.1, 0.2, 1.6, 50  # a wall voxel: |nz| below, metres above the floor between, at least this many voxels to estimate
REFINE_SPAN, REFINE_STEP, REFINE_BIN, REFINE_PTS = 5.0, 0.1, 0.05, 300000  # yaw refinement: deg searched either side of the normals' estimate, deg step, m histogram bin, points used
LANDSCAPE_RATIO = 1.1  # the map is turned to landscape only if it is this much taller than wide (a square one stays as it is)
OBST_LO, OBST_HI = 0.10, 1.80  # m above the floor: the obstacle band measured to pick landscape (the rasterize defaults)
BELOW = 0.3         # m: voxels this far under the chosen plane mean it is not the floor
TRAJ_MIN_LEN, TRAJ_MIN_SHAPE, TRAJ_MAX_ANGLE = 20.0, 0.2, 1.0  # position vote: m, minor/major singular value, deg
YAW_POSES, YAW_MIN_RANGE, YAW_MAX_RATIO, YAW_MAX_ANGLE = 150, 90.0, 0.01, 1.0  # orientation vote: poses used, deg turned, eigenvalue ratio, deg


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


def refine_yaw(X, yaw):
    """Turn (deg) within REFINE_SPAN of `yaw` that piles the points X (N, 2) onto the fewest rows and columns: the sum of squared REFINE_BIN histogram
    counts along x and y after the turn. A wall of length L adds about L squared, so the long walls decide and furniture hardly counts; the mean over wall
    normals that gives `yaw` weights every 10 cm voxel the same and lands between walls and furniture (lab_test_01: 1.3 deg turned, 2.5 deg needed)."""
    if len(X) < 2:
        return float(yaw)                                         # no wall points to sharpen: keep the turn the normals gave
    X = X[::max(1, len(X) // REFINE_PTS)]

    def score(a):
        c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
        return sum(float((np.bincount(((u - u.min()) / REFINE_BIN).astype(int)).astype(float) ** 2).sum()) for u in (c * X[:, 0] - s * X[:, 1], s * X[:, 0] + c * X[:, 1]))

    coarse = np.arange(yaw - REFINE_SPAN, yaw + REFINE_SPAN + 1e-9, REFINE_STEP)
    best = coarse[int(np.argmax([score(a) for a in coarse]))]
    fine = np.arange(best - REFINE_STEP, best + REFINE_STEP + 1e-9, REFINE_STEP / 10)
    return float(fine[int(np.argmax([score(a) for a in fine]))])


def level_to_floor(P, traj=None, min_floor=250, max_below=0.10, min_manhattan=0.7):
    """(Q, info): P rotated so the floor is horizontal and shifted so the floor is z = 0, then, if the walls are Manhattan enough, turned about z
    so the walls lie on the x/y axes and the longer side of the map is x (another 90 deg if needed). info: tilt_deg, resid_std (m), n_floor, below, traj, yaw_deg (turn applied), manhattan (wall strength),
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
    aligned = strength >= min_manhattan
    turn = 0.0
    if aligned:
        z = P @ Rl[2] - d
        turn = refine_yaw((P @ Rl.T)[:, :2][(z > WALL_LO) & (z < WALL_HI)], yaw)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", turn, degrees=True).as_matrix() @ Rl
    T[2, 3] = -d
    if aligned:
        o = P @ T[:3, :3].T + T[:3, 3]
        o = o[(o[:, 2] > OBST_LO) & (o[:, 2] <= OBST_HI)][:, :2]
        ex, ey = np.ptp(np.percentile(o, [1, 99], axis=0), axis=0)
        if ey > LANDSCAPE_RATIO * ex:  # Manhattan fixes the yaw only modulo 90 deg: one more quarter turn keeps the walls on the axes
            turn += 90.0
            T[:3, :3] = Rotation.from_euler("z", turn, degrees=True).as_matrix() @ Rl
    return P @ T[:3, :3].T + T[:3, 3], {"tilt_deg": tilt, "resid_std": float(dist[np.abs(dist) < floor_tilt.TOL].std()), "n_floor": n, "below": below,
                                        "traj": checked, "yaw_deg": turn, "manhattan": strength, "T": T}
