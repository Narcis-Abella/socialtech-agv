"""Run: python3 -B test_run_eval.py"""
import os
import pathlib
import subprocess
import tempfile

SCRIPT = pathlib.Path(__file__).with_name("run_eval.sh")

# ros2 that finds every package, starts "launches" that never come up and reports bt_navigator as inactive forever
ROS2 = """#!/bin/bash
case "$1 $2" in
  "pkg prefix") echo /nonexistent ;;
  "launch "*) exec sleep 20 ;;
  "lifecycle get") echo "inactive [2]" ;;
esac
"""


def test_gives_up_when_nav2_never_activates():
    with tempfile.TemporaryDirectory() as d:
        d = pathlib.Path(d)
        (d / "bin").mkdir()
        (d / "bin" / "ros2").write_text(ROS2)
        (d / "bin" / "ros2").chmod(0o755)
        (d / "ros_env.sh").write_text("")
        env = dict(os.environ, PATH=f"{d / 'bin'}:{os.environ['PATH']}", ROS_ENV=str(d / "ros_env.sh"), SIM_WAIT="0", NAV_TIMEOUT="2")
        r = subprocess.run(["bash", str(SCRIPT), str(d / "out"), "1"], env=env, capture_output=True, text=True, timeout=15)
        assert r.returncode != 0, r.stdout
        assert "bt_navigator not active after 2 s" in r.stderr, r.stderr
        assert (d / "out" / "sim.log").exists()   # `out` is created by the script


if __name__ == "__main__":
    test_gives_up_when_nav2_never_activates()
    print("ok")
