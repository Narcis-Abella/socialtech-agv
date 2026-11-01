"""Test-only synthetic rooms (floor, ceiling, walls, a doorway) with a known floor tilt, shared by the tests of floor_level, grid2d and make_map."""
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

RNG = np.random.default_rng(7)
FLOOR_Z = -0.6    # floor below the sensor (the PLY origin is the sensor, not the floor)
ROOM_H = 2.5      # floor-to-ceiling
TILT = 0.6        # deg, the whole map is rotated about y (what a biased gravity init does)
RES = 0.05


def tilt_rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def room(tilt=TILT, floor_noise=0.01, floor_pts=60000, ceiling_pts=100000, dz=0.0, yaw=0.0):
    """Points of a 10 x 8 m room in its level frame, then tilted as a whole. Walls at x=+-5 and y=+-4; the y=+4 wall has a
    doorway (|x|<1) that only keeps its lintel above 2.0 m. More ceiling than floor points: the ceiling is the bigger plane.
    The floor has a 1 m hole at (2..3, -3..-2): the LiDAR saw nothing there. dz shifts the whole cloud (origin below the floor when dz > 0.6). yaw turns the building about z (before the tilt): a SLAM world not aligned with the walls."""
    pts = []
    f = RNG.uniform([-5, -4], [5, 4], (floor_pts, 2))
    f = f[~((f[:, 0] > 2) & (f[:, 0] < 3) & (f[:, 1] > -3) & (f[:, 1] < -2))]
    floor_pts = len(f)
    pts.append(np.c_[f, FLOOR_Z + RNG.normal(0, floor_noise, floor_pts)])
    c = RNG.uniform([-5, -4], [5, 4], (ceiling_pts, 2))
    pts.append(np.c_[c, np.full(ceiling_pts, FLOOR_Z + ROOM_H)])
    for axis, val in ((0, -5.0), (0, 5.0), (1, -4.0), (1, 4.0)):
        n = 6000
        along = RNG.uniform(-4 if axis == 0 else -5, 4 if axis == 0 else 5, n)
        z = FLOOR_Z + RNG.uniform(0.0, ROOM_H, n)
        if axis == 1 and val == 4.0:
            z = np.where(np.abs(along) < 1.0, FLOOR_Z + RNG.uniform(2.0, ROOM_H, n), z)  # doorway: only the lintel
        pts.append(np.c_[np.full(n, val), along, z] if axis == 0 else np.c_[along, np.full(n, val), z])
    P = np.vstack(pts) + [0, 0, dz]
    return P @ Rotation.from_euler("z", yaw, degrees=True).as_matrix().T @ tilt_rot(tilt).T


def path(tilt=TILT, extra=0.0, straight=False, arc=None):
    """LiDAR poses (positions (N, 3), quaternions xyzw (N, 4)) in the same tilted frame as room(): a 22 m circle at the sensor height
    (z=0 in the level frame) with the robot heading along the path, a 10 m line, or an arc of `arc` degrees. The robot's yaw axis is the floor
    normal; extra tilts the whole trajectory further (a robot that is not parallel to the floor)."""
    yaw = np.linspace(0, np.radians(arc) if arc else 2 * np.pi, 400)
    P = np.c_[np.linspace(-5, 5, 400), np.zeros(400), np.zeros(400)] if straight else np.c_[3.5 * np.cos(yaw), 3.5 * np.sin(yaw), np.zeros(400)]
    if straight:
        yaw = np.zeros(400)
    R = Rotation.from_matrix(tilt_rot(tilt) @ tilt_rot(extra)) * Rotation.from_rotvec(np.c_[np.zeros(400), np.zeros(400), yaw])
    return P @ tilt_rot(extra).T @ tilt_rot(tilt).T, R.as_quat()


def round_room(tilt=TILT):
    """A cylindrical room (walls at every angle), radius 4, with a floor and a ceiling: not a Manhattan building."""
    a = RNG.uniform(0, 2 * np.pi, 20000)
    z = FLOOR_Z + RNG.uniform(0, ROOM_H, 20000)
    wall = np.c_[4 * np.cos(a), 4 * np.sin(a), z]
    disc = lambda n, zz: np.c_[RNG.uniform(-4, 4, (n * 2, 2)), np.full(n * 2, zz)]
    floor, ceiling = disc(30000, FLOOR_Z), disc(50000, FLOOR_Z + ROOM_H)
    keep = lambda d: d[np.hypot(d[:, 0], d[:, 1]) < 4]
    return np.vstack([wall, keep(floor), keep(ceiling)]) @ tilt_rot(tilt).T


def furnished_room(yaw=2.5, panels=12):
    """room(yaw) plus short, dense vertical panels (shelves, partitions) lying on the world axes, i.e. `yaw` deg off the building's walls.
    Their normals are as many as the shell's, so a mean over normals lands in between; the long walls decide a projection histogram."""
    P = room(tilt=0.0, yaw=yaw)
    panel_pts = []
    for i in range(panels):
        c = RNG.uniform([-3.5, -2.5], [3.5, 2.5]); n = 700
        along = RNG.uniform(-0.5, 0.5, n); z = FLOOR_Z + RNG.uniform(0.0, 2.0, n)
        panel_pts.append(np.c_[c[0] + along, np.full(n, c[1]), z] if i % 2 == 0 else np.c_[np.full(n, c[0]), c[1] + along, z])
    return np.vstack([P, np.vstack(panel_pts)]) @ tilt_rot(TILT).T


def extent(Q):
    """(x, y) range of the obstacle points (0.10-1.80 m), 1st-99th percentile."""
    o = Q[(Q[:, 2] > 0.1) & (Q[:, 2] <= 1.8)]
    return np.ptp(np.percentile(o[:, :2], [1, 99], axis=0), axis=0)


def write_test_ply(path, P, intensity=True):
    cols = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")] + ([("intensity", "<f4")] if intensity else [])
    a = np.zeros(len(P), dtype=cols)
    a["x"], a["y"], a["z"] = P[:, 0], P[:, 1], P[:, 2]
    head = "ply\nformat binary_little_endian 1.0\nelement vertex %d\n" % len(P) + "".join(f"property float {n}\n" for n, _ in cols) + "end_header\n"
    Path(path).write_bytes(head.encode() + a.tobytes())
