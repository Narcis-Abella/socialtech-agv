"""grid2d: rasterize, unknown-cell fill and the PGM/YAML writer."""
import tempfile
from pathlib import Path

import numpy as np

import beams_oracle
import floor_level
import grid2d
from synth_corridor import LEN, ORIGIN, RES as CRES, T as CT, cell_at, corridor_grid, drive, planes
from synth_room import RES, RNG, TILT, room, tilt_rot


def cell(grid, origin, x, y, res=RES):
    """Value of the cell containing (x, y); image row 0 is the top (max y), as in map_server PGMs."""
    return grid[grid.shape[0] - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)]


def occupied_near(grid, origin, x, y):
    """Is any cell within one cell of (x, y) occupied? A wall sits on a cell border, so a tiny turn moves it to the neighbour cell."""
    return any(cell(grid, origin, x + dx, y + dy) == grid2d.OCC for dx in (-RES, 0, RES) for dy in (-RES, 0, RES))


def column_sharpness(grid):
    """How concentrated the occupied cells are in few columns/rows (aligned walls pile up in single columns)."""
    occ = grid == grid2d.OCC
    return sum((c.astype(float) ** 2).sum() for c in (occ.sum(0), occ.sum(1))) / occ.sum() ** 2


def speck(x, y, z=0.5):
    """Three diagonal obstacle cells (8-connected) of 4 points each, floating z above the floor: a thin object seen in a few scans. x, y: lower-left corner of a cell."""
    cells = [(x + i * RES, y + i * RES) for i in range(3)]
    return np.array([(cx + dx, cy + dy, z) for cx, cy in cells for dx, dy in ((0.01, 0.01), (0.03, 0.01), (0.01, 0.03), (0.03, 0.03))])


def test_rasterize_keeps_walls_drops_lintel_and_ceiling_and_marks_floor_free():
    Q, _ = floor_level.level_to_floor(room())
    grid, origin = grid2d.rasterize(Q, RES)
    assert occupied_near(grid, origin, 5.0, 0.0)                        # side wall
    assert occupied_near(grid, origin, 3.0, 4.0)                        # wall beside the doorway
    assert not occupied_near(grid, origin, 0.0, 4.0)                    # doorway: the lintel is above 1.80 m
    open_floor = [cell(grid, origin, x, y) for x in np.arange(-3, 1, 0.05) for y in np.arange(-2, 2, 0.05)]
    assert np.mean(np.array(open_floor) == grid2d.FREE) > 0.7        # open floor: free where a floor point fell (about 1.9 per 5 cm cell)
    assert cell(grid, origin, 2.5, -2.5) == grid2d.UNK             # the floor hole: nothing seen there
    occ = (grid == grid2d.OCC).sum()
    assert occ < 0.05 * grid.size, occ


def test_small_gaps_in_explored_floor_are_filled_but_a_big_hole_is_not(monkeypatch):
    Q, _ = floor_level.level_to_floor(room())
    monkeypatch.setattr(grid2d, "CLOSE_M", 0.0)
    raw, origin = grid2d.rasterize(Q, RES)
    monkeypatch.setattr(grid2d, "CLOSE_M", 0.3)
    filled, _ = grid2d.rasterize(Q, RES)
    open_floor = lambda g: np.mean([cell(g, origin, x, y) == grid2d.FREE for x in np.arange(-3, 1, 0.05) for y in np.arange(-2, 2, 0.05)])
    assert open_floor(raw) < 0.9 and open_floor(filled) > 0.97, (open_floor(raw), open_floor(filled))  # the speckle is closed
    assert cell(filled, origin, 2.5, -2.5) == grid2d.UNK            # a 1 m hole is not invented
    assert ((raw == grid2d.OCC) == (filled == grid2d.OCC)).all()


def test_isolated_outliers_do_not_inflate_the_grid():
    P = room()
    far = np.r_[P, RNG.uniform([-100, -100, -0.5], [100, 100, 1.5], (30, 3))]  # 30 stray points scattered far away
    Q, _ = floor_level.level_to_floor(far)
    grid, _ = grid2d.rasterize(Q, RES)
    assert grid.shape[0] < 300 and grid.shape[1] < 300, grid.shape


def test_write_map_makes_pgm_and_map_server_yaml():
    grid = np.full((6, 8), grid2d.UNK, dtype=np.uint8)
    grid[0, 0], grid[5, 7] = grid2d.OCC, grid2d.FREE
    with tempfile.TemporaryDirectory() as tmp:
        grid2d.write_map(Path(tmp) / "m", grid, RES, (-1.5, 2.0), T=np.diag([1.0, 1.0, 1.0, 1.0]))
        raw = (Path(tmp) / "m.pgm").read_bytes()
        assert raw.startswith(b"P5\n8 6\n255\n") and raw[-48:] == grid.tobytes()
        y = (Path(tmp) / "m.yaml").read_text()
        # map_server reads a pixel as free when (255 - pixel) / 255 <= free_thresh: 205 (unknown) is 0.19608, so 0.25 would load unknown as free
        assert "# T_map_world" in y and "[1.0, 0.0, 0.0, 0.0]" in y, y  # the transform from the SLAM world, as a comment (map_server ignores it)
        for line in ("image: m.pgm", "resolution: 0.05", "origin: [-1.5, 2.0, 0.0]", "negate: 0", "occupied_thresh: 0.65", "free_thresh: 0.196"):
            assert line in y, (line, y)


def test_a_prefix_with_a_dot_keeps_its_name_and_the_yaml_points_at_the_pgm():
    grid = np.full((2, 2), grid2d.FREE, dtype=np.uint8)
    with tempfile.TemporaryDirectory() as tmp:
        grid2d.write_map(Path(tmp) / "eco_v1.2", grid, RES, (0.0, 0.0))
        assert sorted(f.name for f in Path(tmp).iterdir()) == ["eco_v1.2.pgm", "eco_v1.2.yaml"]
        assert "image: eco_v1.2.pgm" in (Path(tmp) / "eco_v1.2.yaml").read_text()


def test_obstacle_islands_smaller_than_min_component_become_free_and_walls_stay(monkeypatch):
    Q, _ = floor_level.level_to_floor(room())
    Q = np.r_[Q, speck(-1.0, 0.5)]
    monkeypatch.setattr(grid2d, "ISLAND_CELLS_5CM", 0)
    kept, origin = grid2d.rasterize(Q, RES)
    monkeypatch.setattr(grid2d, "ISLAND_CELLS_5CM", 10)    # island_cells(0.05) == 10
    dropped, _ = grid2d.rasterize(Q, RES)
    assert occupied_near(kept, origin, -0.95, 0.55)                      # with islands off the speck is on the map
    assert not occupied_near(dropped, origin, -0.95, 0.55)
    assert cell(dropped, origin, -0.95, 0.55) == grid2d.FREE         # open floor again, not unknown
    assert ((kept == grid2d.OCC) & (dropped != grid2d.OCC)).sum() == 3   # only the speck's 3 cells went; the walls are big components
    assert ((dropped == grid2d.OCC) & (kept != grid2d.OCC)).sum() == 0


def test_halo_drops_the_thin_fuzz_beside_a_dense_wall_and_nothing_else(monkeypatch):
    rng = np.random.default_rng(5)
    wall = np.c_[rng.uniform(0, 3, 20000), rng.uniform(-0.01, 0.01, 20000), rng.uniform(0.2, 1.5, 20000)]          # wall along x at y = 0, ~170 points per 2.5 cm cell
    fuzz = np.c_[rng.uniform(0.5, 2.5, 200), np.full(200, 0.075), rng.uniform(0.3, 1.0, 200)]                     # 200 points 7.5 cm off the wall: ~2.5 per cell
    lone = np.c_[np.full(8, 2.0), np.full(8, 1.5), np.linspace(0.3, 0.6, 8)]                                  # a thin object far from any wall
    Q = np.vstack([wall, fuzz, lone, [[0, -1, 0.0], [3, 2, 0.0]]])
    for name, value in (("CLOSE_M", 0.0), ("ISLAND_CELLS_5CM", 0)):    # isolate the halo rule
        monkeypatch.setattr(grid2d, name, value)
    monkeypatch.setattr(grid2d, "HALO", (0.0, 0.0))
    plain, origin = grid2d.rasterize(Q, 0.025)
    monkeypatch.setattr(grid2d, "HALO", (0.075, 0.15))
    cleaned, _ = grid2d.rasterize(Q, 0.025)
    at = lambda g, x, y: g[g.shape[0] - 1 - int((y - origin[1]) / 0.025), int((x - origin[0]) / 0.025)] == grid2d.OCC
    assert at(plain, 1.5, 0.075) or at(plain, 1.5, 0.1) or at(plain, 1.5, 0.05)      # the fuzz is occupied without the halo rule
    assert not any(at(cleaned, x, y) for x in np.arange(0.6, 2.4, 0.025) for y in (0.075, 0.1))
    assert at(cleaned, 1.5, 0.0) and at(cleaned, 2.0, 1.5)                            # the wall and the lone thin object stay
    assert ((cleaned == grid2d.OCC) <= (plain == grid2d.OCC)).all()


def test_walls_end_up_on_pixel_rows_and_columns():
    P = room(yaw=23.0)
    Q, info = floor_level.level_to_floor(P)
    assert abs(info["yaw_deg"] + 23.0) < 0.3 and info["manhattan"] > 0.8, info
    aligned, _ = grid2d.rasterize(Q, RES)
    a = np.radians(-info["yaw_deg"])                       # undo the turn level_to_floor applied: the walls go back to ~23 degrees off the pixel axes
    c, s = np.cos(a), np.sin(a)
    Q0 = Q.copy()
    Q0[:, :2] = Q[:, :2] @ np.array([[c, s], [-s, c]])
    skewed, _ = grid2d.rasterize(Q0, RES)
    assert column_sharpness(aligned) > 3 * column_sharpness(skewed), (column_sharpness(aligned), column_sharpness(skewed))


def counter(scans):
    """The beam counter fill_unknown takes: the numpy oracle over analytic scans, the lidar at the IMU origin."""
    return lambda C, margin: beams_oracle.count_beams(C, scans, (0, 0, 0), margin)


def test_the_defaults_are_the_validated_rule():
    assert grid2d.HALO == (0.10, 0.10) and grid2d.CLOSE_M == 0.30 and grid2d.MIN_HITS == 2
    assert grid2d.island_cells(0.05) == 10 and grid2d.island_cells(0.025) == 40       # the same physical speck size at any cell size


def test_fill_unknown_without_unknown_cells_returns_the_same_grid():
    g = np.full((20, 30), grid2d.FREE, np.uint8)
    g[5, 5] = grid2d.OCC
    out = grid2d.fill_unknown(g, CRES, ORIGIN, CT, np.array([[0.0, 0.0, 0.5]]), lambda C, m: (_ for _ in ()).throw(AssertionError("no candidate to count")))
    assert (out == g).all() and out is not g


def test_unknown_floor_the_beams_crossed_is_freed_and_the_outside_stays_unknown():
    g0, X, Y = corridor_grid()
    scans, traj = drive(planes())
    g = grid2d.fill_unknown(g0, CRES, ORIGIN, CT, traj, counter(scans))
    side = (X > 1) & (X < LEN - 1) & (np.abs(Y) > 0.3) & (np.abs(Y) < 0.9)
    assert (g[side] == grid2d.FREE).mean() > 0.95, (g[side] == grid2d.FREE).mean()   # between the path and the walls
    assert (g[np.abs(Y) > 1.2] == grid2d.UNK).all()                              # behind the walls no beam ever went
    known = g0 != grid2d.UNK
    assert (g[known] == g0[known]).all()


def test_a_cell_the_beams_keep_hitting_is_not_freed():
    g0, _, _ = corridor_grid()
    scans, traj = drive(planes(pillar=True))
    g = grid2d.fill_unknown(g0, CRES, ORIGIN, CT, traj, counter(scans))
    assert g[cell_at(5.02, 0.5)] == grid2d.UNK       # on the pillar: hit from both sides
    assert g[cell_at(3.0, 0.5)] == grid2d.FREE


def test_a_few_stray_hits_are_tolerated_but_a_real_share_is_not():
    passed = np.array([[100, 100, 100], [100, 100, 100]])
    hit = np.array([[1, 0, 0], [4, 4, 0]])       # 1 of 301 observations vs 8 of 308
    assert grid2d.free_cells(hit, passed).tolist() == [True, False]


def test_pass_evidence_is_only_required_at_the_heights_that_were_observed():
    hit = np.zeros((3, 3), int)
    passed = np.array([[5, 5, 0],                # two heights seen, the third never: free
                       [5, 0, 0],                # only one height seen: not enough
                       [5, 5, 2]])               # all seen but one with 2 passes: not free
    assert grid2d.free_cells(hit, passed).tolist() == [True, False, False]


def test_only_cells_connected_to_known_free_space_are_kept():
    g = np.full((10, 10), grid2d.UNK, np.uint8)
    g[0, 0] = grid2d.FREE
    m = np.zeros((10, 10), bool)
    m[0, 1:4] = True                             # touches the free cell
    m[7:9, 7:9] = True                           # an isolated pocket
    keep = grid2d.connected_to_free(m, g)
    assert keep[0, 1:4].all() and not keep[7:9, 7:9].any()
