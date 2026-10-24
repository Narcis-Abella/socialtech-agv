"""Run: python3 -B test_worlds.py   (offline: numpy, scipy, PIL, yaml)"""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

import scenarios as sc
import worlds as w

RES = 0.05


def room(extra=None, mapped=True):
    boxes = [{"name": "n", "x": 5.0, "y": 4.0, "yaw": 0.0, "sx": 10.2, "sy": 0.2, "sz": 2.5}, {"name": "s", "x": 5.0, "y": 0.0, "yaw": 0.0, "sx": 10.2, "sy": 0.2, "sz": 2.5},
             {"name": "w", "x": 0.0, "y": 2.0, "yaw": 0.0, "sx": 0.2, "sy": 4.2, "sz": 2.5}, {"name": "e", "x": 10.0, "y": 2.0, "yaw": 0.0, "sx": 0.2, "sy": 4.2, "sz": 2.5}]
    return {"name": "room", "res": RES, "bounds": [-1.0, -1.0, 11.0, 5.0], "seed": [5.0, 2.0], "boxes": boxes + (extra or []), "trials": []}


def test_axis_aligned_box_area_and_position():
    spec = room()
    occ, origin = w.rasterize(spec)
    assert occ.shape == (int(6.0 / RES), int(12.0 / RES)) and origin == (-1.0, -1.0)
    r, c = sc.to_cell(5.0, 4.0, origin, RES, occ.shape[0])
    assert occ[r, c] and not occ[sc.to_cell(5.0, 2.0, origin, RES, occ.shape[0])]
    area = occ.sum() * RES * RES
    assert abs(area - (2 * 10.2 * 0.2 + 2 * 4.2 * 0.2 - 4 * 0.04 * 0)) < 0.35          # four walls, corners overlap slightly


def test_a_rotated_box_has_the_same_area_and_the_right_orientation():
    spec = {"name": "t", "res": RES, "bounds": [0.0, 0.0, 8.0, 8.0], "seed": [1.0, 1.0], "trials": [],
            "boxes": [{"name": "b", "x": 4.0, "y": 4.0, "yaw": math.pi / 4, "sx": 3.0, "sy": 0.4, "sz": 2.0}]}
    occ, origin = w.rasterize(spec)
    assert abs(occ.sum() * RES * RES - 1.2) < 0.1
    r, c = sc.to_cell(4.0 + 0.9, 4.0 + 0.9, origin, RES, occ.shape[0])                   # along the 45 deg axis
    assert occ[r, c]
    r, c = sc.to_cell(4.0 + 0.9, 4.0 - 0.9, origin, RES, occ.shape[0])                   # across it, same distance from the centre
    assert not occ[r, c]


def test_unmapped_boxes_are_in_the_sdf_but_not_in_the_map():
    spec = room([{"name": "box", "x": 5.0, "y": 2.0, "yaw": 0.0, "sx": 0.4, "sy": 0.4, "sz": 1.0, "mapped": False}])
    occ, origin = w.rasterize(spec)
    assert not occ[sc.to_cell(5.0, 2.0, origin, RES, occ.shape[0])]
    models = ET.fromstring(w.sdf(spec)).findall(".//model")
    assert {m.get("name") for m in models} >= {"ground_plane", "b_box", "b_n", "b_s", "b_w", "b_e"}


def test_the_free_region_is_the_closed_room_and_the_outside_is_unknown():
    occ, origin = w.rasterize(room())
    est = w.free_unknown(occ, origin, RES, (5.0, 2.0))
    area = est["free"].sum() * RES * RES
    assert abs(area - 9.8 * 3.8) < 0.8                                                    # the inside of the four walls
    assert not est["free"][sc.to_cell(-0.8, -0.8, origin, RES, occ.shape[0])]              # outside: unknown, not free
    assert not (est["free"] & est["occupied"]).any()


def test_the_sdf_is_well_formed_and_is_the_loyola_world_for_the_simulator():
    root = ET.fromstring(w.sdf(room()))
    assert root.find("world").get("name") == "loyola"
    plugins = {p.get("name") for p in root.findall(".//plugin")}
    assert {"gz::sim::systems::UserCommands", "gz::sim::systems::Sensors", "gz::sim::systems::Physics"} <= plugins


def test_boxes_stand_on_the_ground_with_the_requested_size_and_yaw():
    root = ET.fromstring(w.sdf(room([{"name": "r", "x": 3.0, "y": 1.0, "yaw": 0.5, "sx": 0.4, "sy": 0.6, "sz": 1.0, "mapped": False}])))
    m = next(m for m in root.findall(".//model") if m.get("name") == "b_r")
    x, y, z, _, _, yaw = (float(v) for v in m.find("pose").text.split())
    assert (x, y, z, yaw) == (3.0, 1.0, 0.5, 0.5)                                    # the centre of the box is at half its height: it rests on z = 0
    assert [float(v) for v in m.find(".//collision/geometry/box/size").text.split()] == [0.4, 0.6, 1.0]
    assert m.find("static").text == "true"


def test_door_width_in_the_map_matches_the_spec():
    spec = w.doors_spec()
    occ, origin = w.rasterize(spec)
    h = occ.shape[0]
    col = sc.to_cell(10.0, 0.0, origin, RES, h)[1]
    top, bottom = sc.to_cell(10.0, 5.0, origin, RES, h)[0], sc.to_cell(10.0, -5.0, origin, RES, h)[0]      # only between the north and south walls
    free_rows = np.flatnonzero(~occ[top:bottom, col])
    runs = np.split(free_rows, np.flatnonzero(np.diff(free_rows) > 1) + 1)
    widths = sorted(round(len(r) * RES, 2) for r in runs)
    assert len(widths) == 4
    for got, want in zip(widths, sorted(d["width"] for d in spec["doors"])):
        assert abs(got - want) <= 0.06


def test_scenario_reachability_and_reference_length():
    spec = w.doors_spec()
    out = w.scenario(spec)
    by_id = {t["id"]: t for t in out}
    assert len(out) == 4 and all(t["reachable"] for t in out)
    for t in out:
        assert abs(t["shortest"] - math.hypot(t["goal"][0] - t["start"][0], t["goal"][1] - t["start"][1])) < 0.1      # straight through the door
    narrow = w.scenario(w.doors_spec(widths=(0.4, 0.9, 1.0, 1.2)))
    assert narrow[0]["reachable"] and narrow[0]["shortest"] > 16.5 and abs(narrow[1]["shortest"] - 16.0) < 0.1      # the 0.4 m door is too narrow: the path detours through another one
    assert not any(t["reachable"] and t["shortest"] for t in w.scenario(w.doors_spec(widths=(0.4, 0.4, 0.4, 0.4)))) and w.scenario(w.doors_spec(widths=(0.4, 0.4, 0.4, 0.4)))[0]["shortest"] is None


def test_build_writes_world_map_and_scenario():
    spec = w.doors_spec()
    with tempfile.TemporaryDirectory() as d:
        w.build(spec, d)
        assert os.path.exists(os.path.join(d, "world.sdf")) and os.path.exists(os.path.join(d, "scenario.json"))
        mp = sc.load_map(os.path.join(d, "map.yaml"))
    assert mp["res"] == RES and mp["occ"].any() and (~mp["blocked"]).any()


def test_rows_hall_has_identical_periodic_blocks_and_reachable_lanes():
    spec = w.rows_spec()
    blocks = [b for b in spec["boxes"] if b["name"].startswith("blk")]
    assert len(blocks) == 15 and len({(b["sx"], b["sy"], b["sz"]) for b in blocks}) == 1      # identical
    xs = sorted({b["x"] for b in blocks})
    assert len(xs) == 5 and np.allclose(np.diff(xs), 6.0)                                    # one period along x
    trials = w.scenario(spec)
    assert len(trials) >= 6 and all(t["reachable"] for t in trials)
    assert all(t["shortest"] > 25 for t in trials)                                           # lane to lane across the hall
    occ, origin = w.rasterize(spec)
    for t in trials:
        r, c = sc.to_cell(t["start"][0], t["start"][1], origin, spec["res"], occ.shape[0])
        assert not occ[r, c]


def test_dense_rows_hall_is_as_occupied_as_the_real_maps_and_lanes_are_reachable():
    spec = w.dense_rows_spec()
    occ, origin = w.rasterize(spec)
    h, wd = occ.shape
    inner = occ[int(1.0 / spec["res"]):-int(1.0 / spec["res"]), int(1.0 / spec["res"]):-int(1.0 / spec["res"])]
    share = inner.mean()
    assert 0.12 <= share <= 0.18, share                                     # eco_-1 maps: 14-15 % of the area around the robot
    trials = w.scenario(spec)
    assert len(trials) >= 6 and all(t["reachable"] for t in trials) and all(t["shortest"] > 25 for t in trials)


def test_dyn_hall_walkers_are_unmapped_models_of_the_world_parked_outside_the_walls():
    spec = w.dyn_hall_spec()
    by_name = {"b_" + b["name"]: b for b in spec["boxes"]}
    x0, y0, x1, y1 = spec["bounds"]
    occ, origin = w.rasterize(spec)
    assert occ.sum() > 0 and not occ[occ.shape[0] // 2, occ.shape[1] // 2]            # the hall is mapped and its middle is free
    for t in spec["trials"]:
        assert len(t["dyn"]) == 1
        for d in t["dyn"]:
            b = by_name[d["model"]]                                                     # the model exists in the world
            assert not b["mapped"] and [b["x"], b["y"]] == d["parked"] and not (x0 < b["x"] < x1 and y0 < b["y"] < y1)
            assert abs(d["size"][0] - b["sx"]) < 1e-9 and abs(d["size"][2] - b["sz"]) < 1e-9
            assert len(d["trigger"]) == 3 and d["speed"] >= 0
    assert {"b_walker"} <= {m.get("name") for m in ET.fromstring(w.sdf(spec)).iter("model")}


def test_flip_world_has_a_short_route_through_a_door_and_a_longer_free_one_around_the_wall():
    spec = w.flip_spec()
    trials = {t["id"]: t for t in w.scenario(spec)}                                                                  # with the reference shortest length
    assert set(trials) == {"flip_free", "flip_door"} and "dyn" not in trials["flip_free"] and trials["flip_door"]["dyn"][0]["path"] == [[12.0, 0.0]]
    occ, origin = w.rasterize(spec)
    res = spec["res"]
    cell = lambda x, y: (occ.shape[0] - 1 - int((y - spec["bounds"][1]) / res), int((x - spec["bounds"][0]) / res))
    assert not occ[cell(12.0, 0.0)] and not occ[cell(12.0, 6.0)] and occ[cell(12.0, 3.0)] and occ[cell(12.0, -3.0)]      # door and north gap are open, the wall is not
    s = trials["flip_free"]["shortest"]
    long_len = 2 * (10.0 ** 2 + 6.0 ** 2) ** 0.5
    assert 19.5 < s < 20.5 and long_len > s * 1.1                                                                      # short route ~20 m through the door; the way round is more than 10 % longer


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
