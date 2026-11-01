"""Run: python3 -B test_scan_quality.py   (offline: numpy and scipy)"""
import math

import numpy as np

import scan_quality as q

RES = 0.05


def room():
    """6 x 4 m room, walls 2 cells thick, origin (0, 0), row 0 = top."""
    occ = np.zeros((80, 120), bool)
    occ[:2, :] = occ[-2:, :] = occ[:, :2] = occ[:, -2:] = True
    occ[15:25, 60:70] = True                      # a pillar in the upper half only (y 2.75 .. 3.25 m): the room is not symmetric, so a flipped image row shows
    return occ


def test_endpoints_follow_pose_and_beam_angle():
    pts = q.endpoints(np.array([3.0, 3.0]), angle_min=0.0, inc=math.pi / 2, pose=(1.0, 2.0, math.pi / 2))     # beams at 0 and 90 deg in the body frame, body yawed 90 deg
    assert np.allclose(pts, [[1.0, 5.0], [-2.0, 2.0]])


def test_non_finite_and_infinite_beams_have_no_endpoint():
    r = np.array([2.0, np.inf, np.nan, 1.0])
    pts, idx = q.endpoints(r, 0.0, 0.1, (0.0, 0.0, 0.0), return_index=True)
    assert len(pts) == 2 and list(idx) == [0, 3]


def test_distance_to_the_map_is_small_for_exact_ranges_and_equals_the_error_for_biased_ones():
    occ = room()
    dist = q.distance_map(occ, RES)
    ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    true = np.empty(360)
    for i, a in enumerate(ang):                                                    # exact range to the inner wall surface by marching
        for r in np.arange(0.1, 10, 0.01):
            c, rr = int((3.0 + r * math.cos(a)) / RES), 80 - 1 - int((1.0 + r * math.sin(a)) / RES)
            if occ[rr, c]:
                true[i] = r
                break
    d0 = q.endpoint_map_distance(true, -math.pi, 2 * math.pi / 360, (3.0, 1.0, 0.0), dist, RES, (0.0, 0.0))
    assert np.median(d0[1]) < 0.03 and np.max(d0[1]) < 0.06                         # the max: the pillar subtends only ~4 % of the beams, a percentile would not see a flipped row
    short = q.endpoint_map_distance(true - 0.4, -math.pi, 2 * math.pi / 360, (3.0, 1.0, 0.0), dist, RES, (0.0, 0.0))
    assert abs(np.median(short[1]) - 0.4) < 0.06                                    # 0.4 m too short: it lands 0.4 m inside the free space


def test_endpoints_outside_the_map_are_dropped():
    occ = room()
    dist = q.distance_map(occ, RES)
    r, d = q.endpoint_map_distance(np.array([50.0]), 0.0, 0.1, (3.0, 2.0, 0.0), dist, RES, (0.0, 0.0))
    assert len(r) == 0 and len(d) == 0


def test_band_stats_group_by_range():
    r = np.array([1.0, 1.5, 3.0, 3.5, 7.0, 15.0, 25.0])
    d = np.array([0.01, 0.02, 0.10, 0.50, 0.05, 0.05, 0.06])
    s = q.band_stats(r, d, bands=(0.5, 2.0, 5.0, 10.0, 20.0, 60.0), within=(0.05, 0.10, 0.30))
    assert [b["n"] for b in s] == [2, 2, 1, 1, 1]
    assert s[0]["within"] == [1.0, 1.0, 1.0] and s[1]["within"] == [0.0, 0.5, 0.5]
    assert abs(s[1]["median"] - 0.30) < 1e-9


def test_a_range_on_a_band_edge_belongs_to_the_upper_band():
    s = q.band_stats(np.array([2.0]), np.array([0.01]), bands=(0.5, 2.0, 5.0), within=(0.05,))
    assert [b["n"] for b in s] == [0, 1]


def static_scans(sigma, p_drop, n_scans=60, seed=1):
    rng = np.random.default_rng(seed)
    base = np.linspace(1.0, 12.0, 720)
    out = base + rng.normal(0, sigma, (n_scans, 720))
    out[rng.random((n_scans, 720)) < p_drop] = np.inf
    return out


def test_static_pairs_flags_only_pairs_where_the_reference_did_not_move():
    poses = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.5, 0.001, 0.0], [0.5, 0.001, 0.0004], [0.5, 0.001, 0.1], [np.nan, np.nan, np.nan], [0.5, 0.0, 0.0]])
    assert list(q.static_pairs(poses, pos_thr=0.005, yaw_thr=0.0009)) == [False, True, True, False, False, False]


def test_frame_jitter_recovers_the_beam_noise_and_the_flicker_of_a_static_robot():
    for sigma, p in ((0.02, 0.0), (0.02, 0.05), (0.05, 0.10), (0.02, 0.5)):
        scans = static_scans(sigma, p)
        r, d, flicker = q.frame_jitter(scans, np.ones(len(scans) - 1, bool))
        est = 1.4826 * np.median(np.abs(d - np.median(d))) / math.sqrt(2)
        assert abs(est - sigma) < 0.1 * sigma, (sigma, est)                      # robust sigma of one beam, from the difference of two frames
        assert abs(flicker - 2 * p / (1 + p)) < 0.012, (p, flicker)              # bins finite in exactly one of two frames over bins finite in either


def test_frame_jitter_ignores_pairs_not_flagged_static_and_depth_jumps():
    scans = static_scans(0.02, 0.0, n_scans=10)
    scans[5:] += 0.4                                                               # the world moved after frame 5 (a different surface): |diff| > max_step
    mask = np.ones(9, bool)
    mask[:2] = False
    r, d, _ = q.frame_jitter(scans, mask, max_step=0.3)
    assert len(d) == (9 - 2 - 1) * 720                                              # 2 pairs not static, 1 pair across the jump
    assert np.all(np.abs(d) < 0.3)


def test_flicker_is_reported_per_range_band():
    rng = np.random.default_rng(3)
    scans = np.tile(np.linspace(1.0, 12.0, 720), (80, 1))
    far = np.arange(720) >= 360                                                   # beams beyond 6.5 m lose data 20 % of the time, near ones never
    drop = rng.random((80, 720)) < 0.2
    scans[drop & far[None, :]] = np.inf
    f = q.flicker_bands(scans, np.ones(79, bool), bands=(0.5, 6.0, 20.0))
    assert f[0] == 0.0 and abs(f[1] - 2 * 0.2 / 1.2) < 0.03


def test_frame_jitter_without_static_pairs_is_empty_not_a_crash():
    r, d, f = q.frame_jitter(static_scans(0.02, 0.0, n_scans=5), np.zeros(4, bool))
    assert len(r) == 0 and len(d) == 0 and math.isnan(f)


def test_finite_share_counts_only_finite_beams():
    scans = np.array([[1.0, np.inf, 2.0, np.nan], [1.0, 1.0, 1.0, 1.0]])
    f = q.finite_share(scans)
    assert np.allclose(f, [0.5, 1.0])


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
