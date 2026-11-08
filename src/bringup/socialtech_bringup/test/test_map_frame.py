"""Checks map_frame.py on synthetic trajectories with known answers. Run: python3 test_map_frame.py"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "scripts"))   # the module under test: run by path, not installed as a package

import os
import subprocess
import sys
import tempfile

import numpy as np

import map_frame as pe

RZ90 = np.array([[0, -1, 0, 5.0], [1, 0, 0, 0.0], [0, 0, 1, 0.5], [0, 0, 0, 1.0]])  # p_map = Rz(90) p + (5, 0, .5)
YAML = """image: map.pgm
mode: trinary
resolution: 0.25
origin: [-10.0, -10.0, 0.0]
negate: 0
# T_map_world (row-major; p_map = T p_ply):
# [0.0, -1.0, 0.0, 5.0]
# [1.0, 0.0, 0.0, 0.0]
# [0.0, 0.0, 1.0, 0.5]
# [0.0, 0.0, 0.0, 1.0]
"""


def yaw_quat(y):
    return np.array([0, 0, np.sin(y / 2), np.cos(y / 2)])


def test_planar_moves_poses_into_the_target_frame():
    out = pe.planar(np.array([[1.0, 0, 0]]), np.array([yaw_quat(0.0)]), RZ90)
    assert np.allclose(out, [[5.0, 1.0, np.pi / 2]]), out


def test_map_yaml_without_T_is_refused_and_with_T_is_parsed():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "m.yaml")
        open(p, "w").write("image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
        try:
            pe.read_map_yaml(p)
            raise AssertionError("accepted a YAML without T_map_world")
        except ValueError as e:
            assert "T_map_world" in str(e)
        open(p, "w").write(YAML)
        m = pe.read_map_yaml(p)
        assert m["resolution"] == 0.25 and m["origin"] == (-10.0, -10.0) and np.allclose(m["T_map_world"], RZ90)
        assert m["image"] == os.path.join(d, "map.pgm")


def test_level_rotation_puts_the_floor_normal_on_z_and_roundtrips_as_a_quaternion():
    with tempfile.TemporaryDirectory() as d:
        pitch = np.radians(1.0)  # the IMU frame at start is tilted 1 deg in the gravity-aligned SLAM world
        q0 = np.array([0, np.sin(pitch / 2), 0, np.cos(pitch / 2)])
        np.savetxt(f"{d}/ref.txt", [[0.0, 0, 0, 0, *q0], [0.1, 0, 0, 0, *q0]])
        q = pe.level_quat(f"{d}/ref.txt", RZ90)
        R_L = pe.quat_to_mat(q)
        assert np.allclose(R_L, RZ90[:3, :3] @ pe.quat_to_mat(q0)) and q[3] >= 0
        up_in_fastlio_frame = pe.quat_to_mat(q0).T @ [0, 0, 1]  # world up seen from the IMU frame at start
        assert np.allclose(R_L @ up_in_fastlio_frame, [0, 0, 1])  # RZ90 does not tilt, so up stays up
    for R in (np.eye(3), np.diag([1.0, -1, -1]), np.diag([-1.0, 1, -1]), np.diag([-1.0, -1, 1])):  # all branches (w = 0 included)
        assert np.allclose(pe.quat_to_mat(pe.mat_to_quat(R)), R), R


def test_the_level_command_prints_what_nav_live_reads():
    here = os.path.dirname(os.path.abspath(__file__))
    with tempfile.TemporaryDirectory() as d:
        pitch = np.radians(1.0)
        q0 = np.array([0, np.sin(pitch / 2), 0, np.cos(pitch / 2)])
        np.savetxt(f"{d}/ref.txt", [[0.0, 0, 0, 0, *q0], [0.1, 0, 0, 0, *q0]])
        open(f"{d}/map.yaml", "w").write(YAML)
        out = subprocess.run([sys.executable, os.path.join(here, "..", "scripts", "map_frame.py"), "level", "--ref", f"{d}/ref.txt", "--map", f"{d}/map.yaml"],
                             capture_output=True, text=True, check=True).stdout.split()
        assert len(out) == 4  # "qx qy qz qw", the four numbers `read -r qx qy qz qw` takes in ops/nav_live.sh
        assert np.allclose([float(v) for v in out], pe.level_quat(f"{d}/ref.txt", RZ90), atol=1e-6)


if __name__ == "__main__":
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            v()
    print("ok")
