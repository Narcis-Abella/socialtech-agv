"""Checks make_map.sh with stand-in tools (the real ones are tested on their own). Run: python3 test_make_map.py"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "make_map.sh"

# every stand-in logs its arguments (and VS_SED's content) to $LOG, then does what the real tool does to the files make_map.sh reads
STUBS = {
    "run.sh": 'echo "run $* | sed: $(cat "$VS_SED")" >> "$LOG"\n[ -n "$NO_POSES" ] && exit 0\nmkdir -p "$4/vs/run" && echo "1 0 0 0 0 0 0 1" > "$4/vs/run/alidarState.txt" && : > "$4/node.log"\n',
    "vs_to_ply.sh": 'echo "vs_to_ply $*" >> "$LOG"\n: > "$2"\n',
    "ply_to_map.sh": 'echo "ply_to_map $*" >> "$LOG"\n[ -n "$REJECT" ] && { echo "error: trajectory check failed (position plane): not the floor"; exit 1; }\n: > "$2.pgm"\n',
    "map_visibility.sh": 'echo "map_visibility $*" >> "$LOG"\nprefix=$4\nmkdir -p "$(dirname "$prefix")"\necho P5 > "$prefix.pgm"; echo "image: $(basename "$prefix").pgm" > "$prefix.yaml"\n',
    "report.sh": 'echo "report $*" >> "$LOG"\necho "{}" > "$1/report.json"\n',
}


def run(tmp, out, env_extra=None):
    tools = Path(tmp) / "tools"
    tools.mkdir(exist_ok=True)
    for name, body in STUBS.items():
        (tools / name).write_text("#!/bin/bash\n" + body)
        (tools / name).chmod(0o755)
    env = {**os.environ, "LOG": str(Path(tmp) / "log.txt"), "VOXELSLAM_RUN": str(tools / "run.sh"), "VS_TO_PLY": str(tools / "vs_to_ply.sh"),
           "PLY_TO_MAP": str(tools / "ply_to_map.sh"), "MAP_VISIBILITY": str(tools / "map_visibility.sh"), "MAP_REPORT": str(tools / "report.sh"), **(env_extra or {})}
    return subprocess.run(["bash", str(SCRIPT), "bag dir", str(out)], env=env, capture_output=True, text=True)


def log(tmp):
    return (Path(tmp) / "log.txt").read_text()


def test_the_steps_run_in_order_with_the_arguments_each_tool_needs_and_the_map_lands_in_the_output_dir():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "out with space"      # focus: paths with spaces
        r = run(tmp, out)
        assert r.returncode == 0, r.stderr
        lines = log(tmp).splitlines()
        assert [l.split()[0] for l in lines] == ["run", "vs_to_ply", "ply_to_map", "map_visibility", "report"], lines
        assert lines[0] == f"run bag dir /livox/lidar /livox/imu {out} | sed: s/^\\( *\\)degrade_bound:.*/\\1degrade_bound: 100000/", lines[0]
        assert lines[1] == f"vs_to_ply {out}/vs/run {out}/world.ply 0.05", lines[1]
        assert lines[2] == f"ply_to_map {out}/world.ply {out}/base --ceil 1.80 --traj {out}/vs/run/alidarState.txt", lines[2]
        assert lines[3].startswith(f"map_visibility bag dir {out}/vs/run/alidarState.txt {out}/world.ply ") and lines[3].endswith("--topic /livox/lidar"), lines[3]
        assert (out / "map.pgm").exists() and (out / "report.json").exists()
        assert (out / "map.yaml").read_text().strip() == "image: map.pgm"      # the YAML names the final PGM, not a temporary one


def test_ceil_for_an_outdoor_map_reaches_ply_to_map():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"CEIL": "30"}).returncode == 0
        assert " --ceil 30 " in log(tmp)


def test_a_rejected_floor_stops_the_run_names_the_step_and_leaves_no_map_even_over_an_old_one():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        out.mkdir()
        (out / "map.pgm").write_text("an old map")      # focus: a stale map must not survive a failed run
        (out / "map.yaml").write_text("old")
        r = run(tmp, out, {"REJECT": "1"})
        assert r.returncode != 0
        assert "step 'ply_to_map'" in r.stderr and "not the floor" in r.stderr, r.stderr
        assert not (out / "map.pgm").exists() and not (out / "map.yaml").exists()
        assert "map_visibility" not in log(tmp)


def test_voxel_slam_that_writes_no_poses_fails_at_its_own_step():
    with tempfile.TemporaryDirectory() as tmp:
        r = run(tmp, Path(tmp) / "o", {"NO_POSES": "1"})
        assert r.returncode != 0 and "step 'voxelslam'" in r.stderr, r.stderr
        assert "vs_to_ply" not in log(tmp)


def test_the_flags_make_map_passes_exist_in_the_real_tools():
    ply = subprocess.run([sys.executable, str(HERE / "ply_to_map.py"), "--help"], capture_output=True, text=True).stdout
    vis = subprocess.run([sys.executable, str(HERE / "map_visibility.py"), "--help"], capture_output=True, text=True).stdout
    assert "--ceil" in ply and "--traj" in ply, ply
    assert "--topic" in vis, vis


if __name__ == "__main__":
    test_the_steps_run_in_order_with_the_arguments_each_tool_needs_and_the_map_lands_in_the_output_dir()
    test_ceil_for_an_outdoor_map_reaches_ply_to_map()
    test_a_rejected_floor_stops_the_run_names_the_step_and_leaves_no_map_even_over_an_old_one()
    test_voxel_slam_that_writes_no_poses_fails_at_its_own_step()
    test_the_flags_make_map_passes_exist_in_the_real_tools()
    print("ok")
