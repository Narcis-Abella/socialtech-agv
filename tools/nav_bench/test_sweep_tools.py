"""Checks overlay.py and summarize.py (no ROS needed). Run: python3 test_sweep_tools.py"""
import json
import os
import tempfile

import overlay
import summarize


def test_overlay_keeps_types_and_is_a_ros_parameter_yaml():
    y = overlay.overlay(["alpha1=0.05", "max_beams=120", "update_min_d=1.0", "do_beamskip=false", "robot_model_type=nav2_amcl::DifferentialMotionModel"])
    assert y == ('/**:\n  ros__parameters:\n    use_sim_time: true\n    alpha1: 0.05\n    max_beams: 120\n    update_min_d: 1.0\n    do_beamskip: false\n'
                 '    robot_model_type: "nav2_amcl::DifferentialMotionModel"\n'), y
    # no overrides: `ros__parameters:` with nothing under it is a YAML null that rcl refuses to parse ("No value at line 2"), so a real parameter is always emitted
    assert overlay.overlay([]) == "/**:\n  ros__parameters:\n    use_sim_time: true\n"


def test_summarize_makes_one_row_per_run_and_marks_a_run_without_report():
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(f"{d}/a"), os.makedirs(f"{d}/b")
        json.dump({"converged": True, "convergence_s": 31.0, "pos_err_m": {"median": 0.14, "p95": 0.28, "max": 0.31}, "yaw_err_deg": {"median": 2.5, "p95": 7.5, "max": 9.0},
                   "losses": 0, "map_odom_jumps": 44, "correction_at_robot_cm": {"median": 3.0, "p95": 9.0}, "amcl_std_xy_m": {"median": 0.79, "max": 1.06}}, open(f"{d}/a/report.json", "w"))
        open(f"{d}/a/launch.log", "w").write("x\n[amcl-1] [WARN] Message Filter dropping message: frame 'base_footprint'\nx\n[amcl-1] [WARN] Message Filter dropping message: frame 'base_footprint'\n")
        t = summarize.table(d).splitlines()
        assert t[0].split()[:3] == ["run", "conv_s", "pos_med"] and t[0].split()[-1] == "drops", t[0]
        assert t[1].split() == ["a", "31", "0.14", "0.28", "0.31", "7.5", "0", "44", "9.0", "0.79", "2"], t[1]
        assert t[2].split() == ["b", "NO", "REPORT"], t[2]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
