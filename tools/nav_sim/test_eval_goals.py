"""Run (with ROS sourced, for tf2_ros): python3 -B test_eval_goals.py"""
import numpy as np

import eval_goals as e


def test_clearance():
    g = np.zeros((20, 20), bool)   # row 0 = top (PGM order); free everywhere
    g[10, 14] = True               # occupied cell
    # resolution 0.1, origin (0,0): cell (row 10, col 14) -> x=1.45, y=(20-10-0.5)*0.1=0.95
    assert abs(e.clearance(g, 0.1, (0.0, 0.0), 1.45, 0.45) - 0.5) < 1e-9
    assert e.clearance(g, 0.1, (0.0, 0.0), 0.15, 1.85) == float("inf")


def test_summarize():
    rows = [dict(ok=True, t=10.0, clr=0.5, err=0.1), dict(ok=True, t=20.0, clr=0.3, err=0.3), dict(ok=False, t=30.0, clr=0.2, err=0.9)]
    s = e.summarize(rows)
    assert s["n"] == 3 and s["succeeded"] == 2
    assert s["time_median"] == 15.0 and s["time_max"] == 20.0
    assert s["clearance_min"] == 0.2 and s["err_max"] == 0.9


def test_robot_xy_reads_map_to_base_footprint():
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer
    buf = Buffer()
    assert e.robot_xy(buf) is None     # nothing published yet
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id, t.header.stamp.sec = "map", "base_footprint", 1
    t.transform.translation.x, t.transform.translation.y, t.transform.rotation.w = 3.0, 4.0, 1.0
    buf.set_transform(t, "test")
    assert e.robot_xy(buf) == (3.0, 4.0)


if __name__ == "__main__":
    test_clearance()
    test_summarize()
    test_robot_xy_reads_map_to_base_footprint()
    print("ok")
