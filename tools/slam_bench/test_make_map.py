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
    "run.sh": 'echo "run $* | sed: $(cat "$VS_SED") | rate: $PLAY_RATE" >> "$LOG"\n[ -n "$NO_POSES" ] && exit 0\n[ -e "$4/vs/run" ] && { echo "Cannot create save directory: $4/vs/run/" >&2; exit 1; }\nmkdir -p "$4/vs/run" && echo "1 0 0 0 0 0 0 1" > "$4/vs/run/alidarState.txt" && : > "$4/node.log"\n',
    "vs_to_ply.sh": 'echo "vs_to_ply $*" >> "$LOG"\n: > "$2"\n',
    "ply_to_map.sh": 'echo "ply_to_map $*" >> "$LOG"\n[ -n "$REJECT" ] && { echo "error: trajectory check failed (position plane): not the floor"; exit 1; }\n: > "$2.pgm"\n',
    "map_visibility.sh": 'echo "map_visibility $*" >> "$LOG"\nprefix=$4\nmkdir -p "$(dirname "$prefix")"\necho P5 > "$prefix.pgm"; echo "image: $(basename "$prefix").pgm" > "$prefix.yaml"\nwhile [ $# -gt 0 ]; do [ "$1" = --ply-out ] && : > "$2"; shift; done\n',
    "report.sh": 'echo "report $*" >> "$LOG"\n[ -n "$REPORT_FAIL" ] && exit 1\necho "{}" > "$1/report.json"\n',
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
        assert lines[0] == f"run bag dir /livox/lidar /livox/imu {out} | sed: s/^\\( *\\)degrade_bound:.*/\\1degrade_bound: 100000/ | rate: 1.0", lines[0]
        assert lines[1] == f"vs_to_ply {out}/vs/run {out}/world.ply 0.025", lines[1]
        assert lines[2] == f"ply_to_map {out}/world.ply {out}/base --ceil 1.80 --res 0.025 --traj {out}/vs/run/alidarState.txt", lines[2]
        assert lines[3].startswith(f"map_visibility bag dir {out}/vs/run/alidarState.txt {out}/world.ply ") and lines[3].endswith(f"--topic /livox/lidar --ceil 1.80 --res 0.025 --ply-out {out}/.build/map_clean.ply"), lines[3]
        assert (out / "map.pgm").exists() and (out / "map_clean.ply").exists() and (out / "report.json").exists()   # the cleaned cloud is the one that matches the PGM
        assert (out / "map.yaml").read_text().strip() == "image: map.pgm"      # the YAML names the final PGM, not a temporary one


def test_the_replay_rate_defaults_to_1x_and_can_be_changed():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"PLAY_RATE": "3"}).returncode == 0
        assert "| rate: 3" in log(tmp)


def test_res_sets_the_cloud_thinning_and_the_cell_size_of_every_step_and_voxel_can_still_differ():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"RES": "0.05"}).returncode == 0
        by_tool = {l.split()[0]: l for l in log(tmp).splitlines()}
        assert by_tool["vs_to_ply"].endswith(" 0.05") and " --res 0.05 " in by_tool["ply_to_map"] and " --res 0.05 " in by_tool["map_visibility"], by_tool
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"RES": "0.05", "VOXEL": "0.03"}).returncode == 0
        by_tool = {l.split()[0]: l for l in log(tmp).splitlines()}
        assert by_tool["vs_to_ply"].endswith(" 0.03") and " --res 0.05 " in by_tool["map_visibility"], by_tool


def test_ceil_for_an_outdoor_map_reaches_ply_to_map_and_the_map_that_is_shipped():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"CEIL": "30"}).returncode == 0
        by_tool = {l.split()[0]: l for l in log(tmp).splitlines()}
        assert " --ceil 30 " in by_tool["ply_to_map"] and " --ceil 30 " in by_tool["map_visibility"], by_tool


def test_a_rejected_floor_stops_the_run_names_the_step_and_leaves_no_map_even_over_an_old_one():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        out.mkdir()
        (out / "map.pgm").write_text("an old map")      # focus: a stale map must not survive a failed run
        (out / "map.yaml").write_text("old")
        (out / "map_clean.ply").write_text("old")
        r = run(tmp, out, {"REJECT": "1"})
        assert r.returncode != 0
        assert "step 'ply_to_map'" in r.stderr and "not the floor" in r.stderr, r.stderr
        assert not (out / "map.pgm").exists() and not (out / "map.yaml").exists() and not (out / "map_clean.ply").exists()
        assert "map_visibility" not in log(tmp)


def test_poses_of_an_earlier_run_do_not_pass_for_this_one():
    """voxelslam_run.sh decides the run is over from the age of alidarState.txt: an old one would end it at once and feed an old map."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        (out / "node.log").write_text("resetting system\n")
        r = run(tmp, out, {"NO_POSES": "1"})      # this run writes nothing
        assert r.returncode != 0 and "step 'voxelslam'" in r.stderr, r.stderr
        assert "vs_to_ply" not in log(tmp) and not (out / "node.log").exists()


def test_a_second_run_in_the_same_output_dir_works():
    """Voxel-SLAM exits when its save directory (<out>/vs/run) exists, so make_map.sh clears the one of the earlier run."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        assert run(tmp, out).returncode == 0
        (out / "vs" / "run" / "0.pcd").write_text("an earlier scan")
        r = run(tmp, out)
        assert r.returncode == 0, r.stderr
        assert not (out / "vs" / "run" / "0.pcd").exists() and (out / "map.pgm").exists()


def test_a_run_directory_that_cannot_be_removed_stops_at_its_own_step():
    if os.geteuid() == 0:
        return      # root can remove anything
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs").chmod(0o555)               # an earlier run as another user: its directory cannot be removed
        try:
            r = run(tmp, out)
        finally:
            (out / "vs").chmod(0o755)
        assert r.returncode != 0 and "step 'cleanup'" in r.stderr, r.stderr
        assert not (Path(tmp) / "log.txt").exists()      # no tool ran, in particular not Voxel-SLAM


def test_a_failing_report_leaves_no_map():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        r = run(tmp, out, {"REPORT_FAIL": "1"})
        assert r.returncode != 0 and "step 'report'" in r.stderr, r.stderr
        assert not (out / "map.pgm").exists() and not (out / "map.yaml").exists() and not (out / "map_clean.ply").exists()


def test_voxel_slam_that_writes_no_poses_fails_at_its_own_step():
    with tempfile.TemporaryDirectory() as tmp:
        r = run(tmp, Path(tmp) / "o", {"NO_POSES": "1"})
        assert r.returncode != 0 and "step 'voxelslam'" in r.stderr, r.stderr
        assert "vs_to_ply" not in log(tmp)


VIS_STUB = """import argparse, os, sys
open(os.environ["LOG"], "a").write("map_visibility_py " + " ".join(sys.argv[1:]) + "\\n")
if "--check-engine" in sys.argv:
    if os.environ.get("STUB_CHECK_FAIL"):
        print("map_visibility: the C++ engine is not available here", file=sys.stderr)
    sys.exit(1 if os.environ.get("STUB_CHECK_FAIL") else 0)
ap = argparse.ArgumentParser()
for name in ("bag", "poses", "ply", "prefix"):
    ap.add_argument(name)
for flag in ("--engine", "--topic", "--ceil", "--res", "--ply-out"):
    ap.add_argument(flag)
a = ap.parse_args()
os.makedirs(os.path.dirname(a.prefix), exist_ok=True)
open(a.prefix + ".pgm", "w").write("P5\\n")
open(a.prefix + ".yaml", "w").write("image: " + os.path.basename(a.prefix) + ".pgm\\n")
open(a.ply_out, "w").close()
"""


def run_default_cleaning(tmp, out, env_extra=None):
    """make_map.sh copied next to a stand-in map_visibility.py, with MAP_VISIBILITY left unset: runs the script's own default for the cleaning step."""
    tools = Path(tmp) / "tools"
    tools.mkdir(exist_ok=True)
    for name, body in STUBS.items():
        (tools / name).write_text("#!/bin/bash\n" + body)
        (tools / name).chmod(0o755)
    copy = Path(tmp) / "copy"
    copy.mkdir()
    (copy / "make_map.sh").write_text(SCRIPT.read_text())
    (copy / "map_visibility.py").write_text(VIS_STUB)
    env = {**{k: v for k, v in os.environ.items() if k != "MAP_VISIBILITY"}, "LOG": str(Path(tmp) / "log.txt"), "VOXELSLAM_RUN": str(tools / "run.sh"),
           "VS_TO_PLY": str(tools / "vs_to_ply.sh"), "PLY_TO_MAP": str(tools / "ply_to_map.sh"), "MAP_REPORT": str(tools / "report.sh"), **(env_extra or {})}
    return subprocess.run(["bash", str(copy / "make_map.sh"), "bag dir", str(out)], env=env, capture_output=True, text=True)


def test_the_cleaning_step_counts_beams_in_cpp_by_default_and_the_env_can_still_choose_numpy():
    with tempfile.TemporaryDirectory() as tmp:
        r = run_default_cleaning(tmp, Path(tmp) / "o")
        assert r.returncode == 0, r.stderr
        line = [l for l in log(tmp).splitlines() if l.startswith("map_visibility_py") and "--check-engine" not in l][0]
        assert " --engine cpp " in line, line
    with tempfile.TemporaryDirectory() as tmp:
        r = run_default_cleaning(tmp, Path(tmp) / "o", {"MAP_VISIBILITY": f"python3 {tmp}/copy/map_visibility.py --engine py"})
        assert r.returncode == 0, r.stderr
        lines = [l for l in log(tmp).splitlines() if l.startswith("map_visibility_py")]
        assert len(lines) == 1 and "--check-engine" not in lines[0], lines     # an explicit MAP_VISIBILITY is the user's choice: no preflight
        assert "--engine py" in lines[0] and "--engine cpp" not in lines[0], lines


def test_a_host_without_the_cpp_library_stops_before_voxel_slam_with_the_way_out():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        r = run_default_cleaning(tmp, out, {"STUB_CHECK_FAIL": "1"})
        assert r.returncode != 0 and "FAILED at step 'preflight'" in r.stderr and "not available" in r.stderr, r.stderr    # the tool's own message reaches the user
        lines = log(tmp).splitlines()
        assert [l.split()[0] for l in lines] == ["map_visibility_py"] and "--check-engine" in lines[0], lines      # Voxel-SLAM never started
        assert not (out / "map.pgm").exists()


def test_the_flags_make_map_passes_exist_in_the_real_tools():
    ply = subprocess.run([sys.executable, str(HERE / "ply_to_map.py"), "--help"], capture_output=True, text=True).stdout
    vis = subprocess.run([sys.executable, str(HERE / "map_visibility.py"), "--help"], capture_output=True, text=True).stdout
    assert "--ceil" in ply and "--traj" in ply, ply
    assert "--topic" in vis and "--ply-out" in vis and "--ceil" in vis, vis


if __name__ == "__main__":
    test_the_steps_run_in_order_with_the_arguments_each_tool_needs_and_the_map_lands_in_the_output_dir()
    test_the_replay_rate_defaults_to_1x_and_can_be_changed()
    test_ceil_for_an_outdoor_map_reaches_ply_to_map_and_the_map_that_is_shipped()
    test_a_rejected_floor_stops_the_run_names_the_step_and_leaves_no_map_even_over_an_old_one()
    test_poses_of_an_earlier_run_do_not_pass_for_this_one()
    test_a_second_run_in_the_same_output_dir_works()
    test_a_run_directory_that_cannot_be_removed_stops_at_its_own_step()
    test_a_failing_report_leaves_no_map()
    test_voxel_slam_that_writes_no_poses_fails_at_its_own_step()
    test_the_cleaning_step_counts_beams_in_cpp_by_default_and_the_env_can_still_choose_numpy()
    test_a_host_without_the_cpp_library_stops_before_voxel_slam_with_the_way_out()
    test_the_flags_make_map_passes_exist_in_the_real_tools()
    print("ok")
