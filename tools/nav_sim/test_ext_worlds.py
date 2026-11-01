"""Run: python3 -B test_ext_worlds.py   (offline)"""
import math
import os
import tempfile

import ext_worlds as ew

SDF = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w1">
  <model name="ground_plane"><static>true</static><link name="g"><pose>0 0 -0.025 0 0 0</pose>
    <collision name="g_col"><geometry><box><size>100 100 0.05</size></box></geometry></collision></link></model>
  <model name="W_structure"><static>true</static><pose>0 0 0 0 0 0</pose><link name="l"><pose>0 0 0 0 0 0</pose>
    <collision name="a_col"><pose>2.0 3.0 1.0 0 0 0</pose><geometry><box><size>4.0 0.2 2.0</size></box></geometry></collision>
    <visual name="a_vis"><pose>2.0 3.0 1.0 0 0 0</pose><geometry><box><size>4.0 0.2 2.0</size></box></geometry></visual>
    <collision name="b_col"><pose>-1.0 0.5 0.75 0 0 1.5707963</pose><geometry><box><size>2.0 0.1 1.5</size></box></geometry></collision>
  </link></model>
</world></sdf>"""


def write(txt):
    d = tempfile.mkdtemp()
    p = os.path.join(d, "w.sdf")
    open(p, "w").write(txt)
    return p


def test_reads_only_structure_boxes_with_their_world_pose_and_yaw():
    boxes = ew.parse_boxes(write(SDF))
    assert len(boxes) == 2                                        # the ground plane and the visuals are not boxes of the map
    a, b = boxes
    assert (a["x"], a["y"], a["sx"], a["sy"], a["sz"], a["yaw"]) == (2.0, 3.0, 4.0, 0.2, 2.0, 0.0)
    assert abs(b["yaw"] - math.pi / 2) < 1e-6 and b["x"] == -1.0 and b["sz"] == 1.5


def test_a_box_that_does_not_stand_on_the_floor_is_refused():
    floating = SDF.replace("2.0 3.0 1.0 0 0 0</pose><geometry><box><size>4.0 0.2 2.0", "2.0 3.0 1.9 0 0 0</pose><geometry><box><size>4.0 0.2 2.0", 1)
    try:
        ew.parse_boxes(write(floating))
    except ValueError as e:
        assert "floor" in str(e)
        return
    raise AssertionError("must refuse")


def test_a_roll_or_pitch_is_refused():
    tilted = SDF.replace("-1.0 0.5 0.75 0 0 1.5707963", "-1.0 0.5 0.75 0.2 0 1.5707963", 1)
    try:
        ew.parse_boxes(write(tilted))
    except ValueError as e:
        assert "roll" in str(e) or "tilt" in str(e)
        return
    raise AssertionError("must refuse")


def test_spec_bounds_cover_all_boxes_and_the_seed_is_the_first_spawn():
    spec = ew.spec_from_boxes("w1", ew.parse_boxes(write(SDF)), spawns=[{"id": "S1", "x": 2.0, "y": 1.0, "yaw": 0.0}])
    x0, y0, x1, y1 = spec["bounds"]
    assert x0 < -2.0 and x1 > 4.0 and y0 < 0.0 and y1 > 3.1
    assert spec["seed"] == [2.0, 1.0] and spec["name"] == "w1"
    assert len(spec["boxes"]) == 2 and spec["boxes"][0]["mapped"] is True


def test_a_finer_resolution_doubles_the_cells_and_keeps_the_extent():
    import worlds
    boxes = ew.parse_boxes(write(SDF))
    a = ew.spec_from_boxes("w1", boxes, spawns=[{"id": "S1", "x": 2.0, "y": 1.0, "yaw": 0.0}])
    b = ew.spec_from_boxes("w1", boxes, spawns=[{"id": "S1", "x": 2.0, "y": 1.0, "yaw": 0.0}], res=0.025)
    (oa, _), (ob, _) = worlds.rasterize(a), worlds.rasterize(b)
    assert b["res"] == 0.025 and a["bounds"] == b["bounds"]
    assert abs(ob.shape[0] / oa.shape[0] - 2) < 0.05 and abs(ob.shape[1] / oa.shape[1] - 2) < 0.05
    assert abs(ob.sum() * 0.025 ** 2 - oa.sum() * 0.05 ** 2) / (oa.sum() * 0.05 ** 2) < 0.05     # same obstacle area


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
