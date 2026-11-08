"""Checks of the pure parts of send_goal.py (no node is started, but send_goal.py imports rclpy and nav2_msgs: run them in the robot image)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))   # the module under test: run by path (pytest gets it from conftest.py)
try:
    from send_goal import phases, pick, wrap
except ModuleNotFoundError as e:      # rclpy / nav2_msgs: only in the robot image. A missing NAME (ImportError) is a real failure and must not skip
    if __name__ != "__main__":
        raise      # under pytest, conftest.py leaves these files out when the image is missing; getting here is a real failure
    print(f"skip: needs the robot image ({e})")
    raise SystemExit(0)

assert abs(wrap(3 * math.pi) - math.pi) < 1e-9 or abs(wrap(3 * math.pi) + math.pi) < 1e-9

goals = [{"x": 3.0, "y": 0.0}, {"x": -3.0, "y": 0.0}, {"x": 0.5, "y": 0.0}, {"x": 9.0, "y": 0.0}]
assert pick(goals, 0, 0, 0.0) == 1            # facing +x: the goal behind needs the biggest turn; 0.5 m and 9 m are out of range
assert pick(goals, 0, 0, math.pi) == 0        # facing -x: now it is the one ahead on +x
assert pick([{"x": 0.2, "y": 0.0}], 0, 0, 0.0) is None

# 2 s turning in place at 0.2 rad/s, then it drives: the base reports 0.15 rad/s
cmd = [(10 + i * 0.05, 0.0, 0.2) for i in range(41)] + [(12.05 + i * 0.05, 0.3, 0.0) for i in range(10)]
odom = [(10 + i * 0.02, 0.15) for i in range(110)]
p = phases(cmd, odom, 9.9)
assert abs(p["first_translation_s"] - 2.15) < 1e-6
assert abs(p["turn_s"] - 2.0) < 1e-6 and abs(p["mean_w"] - 0.2) < 1e-9 and abs(p["peak_w"] - 0.2) < 1e-9
assert abs(p["base_turned_deg"] - math.degrees(0.15 * 2.0)) < 1.0
assert "turn_s" not in phases([(10.0, 0.3, 0.0), (10.1, 0.3, 0.0)], [], 9.9)   # straight away
print("ok")
