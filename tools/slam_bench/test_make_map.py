"""Checks make_map.sh with stand-in tools (the real ones are tested on their own). Run: python3 -m pytest test_make_map.py"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "make_map.sh"

# every stand-in logs its arguments (and VS_SED's content) to $LOG, then does what the real tool does to the files make_map.sh reads
STUBS = {
    "run.sh": 'echo "run $* | sed: $(cat "$VS_SED") | rate: $PLAY_RATE" >> "$LOG"\n[ -n "$NO_POSES" ] && exit 0\n[ -e "$4/vs/run" ] && { echo "Cannot create save directory: $4/vs/run/" >&2; exit 1; }\nmkdir -p "$4/vs/run"\necho "1 0 0 0 0 0 0 1" > "$4/vs/run/alidarState.txt"\necho "resetting system" > "$4/node.log"\necho edge > "$4/vs/edge.txt"\n',
    "engine.sh": 'echo "engine $*" >> "$LOG"\n[ -n "$STUB_CHECK_FAIL" ] && { echo "ghosts: the C++ beam engine is not available here" >&2; exit 1; }\nexit 0\n',
    "make_map_py.sh": ('echo "make_map $*" >> "$LOG"\n'
                       'while [ $# -gt 0 ]; do [ "$1" = --out ] && out=$2; shift; done\n'
                       '[ -n "$REJECT" ] && { echo "make_map: floor tilt: the plane is not the floor"; exit 1; }\n'
                       'echo ply > "$out/map.ply"; echo clean > "$out/map_clean.ply"; echo P5 > "$out/map.pgm"; echo "image: map.pgm" > "$out/map.yaml"\n'
                       '[ -n "$FAIL_LATE" ] && { echo "make_map: report failed"; exit 1; }\n'
                       'echo "{\\"resets\\": 0, \\"end_to_start_m\\": 0.1, \\"sensor_height_m\\": 0.5}" > "$out/report.json"\n'
                       '[ -n "$STUB_WARN" ] && echo "WARNING: only 90 of 100 scans of the bag got a pose: relaunch in real time, PLAY_RATE=1 ops/make_map.sh BAG MAP"\nexit 0\n'),
}


def run(tmp, out, env_extra=None, args=()):
    tools = Path(tmp) / "tools"
    tools.mkdir(exist_ok=True)
    for name, body in STUBS.items():
        (tools / name).write_text("#!/bin/bash\n" + body)
        (tools / name).chmod(0o755)
    env = {**{k: v for k, v in os.environ.items() if k != "PLAY_RATE"}, "LOG": str(Path(tmp) / "log.txt"), "VOXELSLAM_RUN": str(tools / "run.sh"), "ENGINE_CHECK": str(tools / "engine.sh"),
           "MAKE_MAP_PY": str(tools / "make_map_py.sh"), **(env_extra or {})}
    return subprocess.run(["bash", str(SCRIPT), *args, "bag dir", str(out)], env=env, capture_output=True, text=True)


def log(tmp):
    f = Path(tmp) / "log.txt"
    return f.read_text() if f.exists() else ""      # no tool ran


def tools_logged(tmp):
    return [l.split()[0] for l in log(tmp).splitlines()]


def test_the_steps_run_in_order_with_the_arguments_each_tool_needs_and_the_outputs_land_in_the_output_dir():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "out with space"      # focus: paths with spaces
        r = run(tmp, out)
        assert r.returncode == 0, r.stderr
        lines = log(tmp).splitlines()
        assert [l.split()[0] for l in lines] == ["engine", "run", "make_map"], lines
        assert lines[0].strip() == "engine"
        assert lines[1] == f"run bag dir /livox/lidar /livox/imu {out} | sed: s/^\\( *\\)degrade_bound:.*/\\1degrade_bound: 100000/ | rate: 2", lines[1]
        assert lines[2] == f"make_map --run {out}/vs/run --bag bag dir --out {out} --res 0.025 --voxel 0.025 --ceil 1.80 --topic /livox/lidar --play-rate 2", lines[2]
        assert all((out / f).exists() for f in ("map.pgm", "map.yaml", "map.ply", "map_clean.ply", "report.json"))
        assert "make_map: OK" in r.stdout


def test_the_replay_rate_defaults_to_2x_and_can_be_changed():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"PLAY_RATE": "1"}).returncode == 0
        assert "| rate: 1" in log(tmp) and "--play-rate 1" in log(tmp)


def test_res_voxel_and_ceil_reach_make_map():
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"RES": "0.05", "CEIL": "30"}).returncode == 0
        assert "--res 0.05 --voxel 0.05 --ceil 30 " in log(tmp), log(tmp)         # the cloud is thinned like the cells unless VOXEL says otherwise
    with tempfile.TemporaryDirectory() as tmp:
        assert run(tmp, Path(tmp) / "o", {"RES": "0.05", "VOXEL": "0.03"}).returncode == 0
        assert "--res 0.05 --voxel 0.03 " in log(tmp), log(tmp)


def test_a_rejected_floor_stops_the_run_names_the_step_and_leaves_no_map_even_over_an_old_one():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        out.mkdir()
        for f in ("map.pgm", "map.yaml", "map_clean.ply", "map.ply", "report.json"):
            (out / f).write_text("an old one")      # focus: a stale map must not survive a failed run
        r = run(tmp, out, {"REJECT": "1"})
        assert r.returncode != 0
        assert "step 'make_map'" in r.stderr and "not the floor" in r.stderr, r.stderr
        assert not any((out / f).exists() for f in ("map.pgm", "map.yaml", "map_clean.ply", "report.json"))


def test_a_failure_after_the_map_was_written_leaves_no_map():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        r = run(tmp, out, {"FAIL_LATE": "1"})
        assert r.returncode != 0 and "step 'make_map'" in r.stderr, r.stderr
        assert not any((out / f).exists() for f in ("map.pgm", "map.yaml", "map_clean.ply", "report.json"))


def test_poses_of_an_earlier_run_do_not_pass_for_this_one():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        (out / "vs" / "edge.txt").write_text("an earlier run")       # voxelslam_run.sh waits for this file: an old one would end the wait at once
        (out / "node.log").write_text("resetting system\n")
        r = run(tmp, out, {"NO_POSES": "1"})      # this run writes nothing
        assert r.returncode != 0 and "step 'voxelslam'" in r.stderr, r.stderr
        assert "make_map" not in tools_logged(tmp) and not (out / "node.log").exists() and not (out / "vs" / "edge.txt").exists()


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
        assert log(tmp) == ""      # no tool ran, in particular not Voxel-SLAM


def test_voxel_slam_that_writes_no_poses_fails_at_its_own_step():
    with tempfile.TemporaryDirectory() as tmp:
        r = run(tmp, Path(tmp) / "o", {"NO_POSES": "1"})
        assert r.returncode != 0 and "step 'voxelslam'" in r.stderr, r.stderr
        assert "make_map" not in tools_logged(tmp)


def test_a_host_without_the_cpp_library_stops_before_voxel_slam_with_the_way_out():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        r = run(tmp, out, {"STUB_CHECK_FAIL": "1"})
        assert r.returncode != 0 and "FAILED at step 'preflight'" in r.stderr and "not available" in r.stderr, r.stderr      # the tool's own message reaches the user
        assert tools_logged(tmp) == ["engine"], log(tmp)      # Voxel-SLAM never started
        assert not (out / "map.pgm").exists()


def test_reuse_slam_skips_the_replay_and_builds_from_map_ply_when_it_exists():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        (out / "map.ply").write_text("the raw cloud of that run")
        (out / "node.log").write_text("resetting system\n")
        r = run(tmp, out, args=["--reuse-slam"])
        assert r.returncode == 0, r.stderr
        assert tools_logged(tmp) == ["engine", "make_map"], log(tmp)            # no replay
        assert f"make_map --cloud {out}/map.ply --poses {out}/vs/run/alidarState.txt --bag bag dir" in log(tmp), log(tmp)
        assert (out / "node.log").exists() and (out / "vs" / "run" / "alidarState.txt").exists()      # the SLAM output is kept


def test_reuse_slam_without_map_ply_assembles_the_cloud_from_the_frames_and_without_poses_it_stops():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        (out / "vs" / "run" / "0.pcd").write_text("frame")
        assert run(tmp, out, args=["--reuse-slam"]).returncode == 0
        assert f"make_map --run {out}/vs/run " in log(tmp)
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        r = run(tmp, out, args=["--reuse-slam"])
        assert r.returncode != 0 and "step 'reuse'" in r.stderr, r.stderr
        assert "run" not in tools_logged(tmp) and "make_map" not in tools_logged(tmp)
    with tempfile.TemporaryDirectory() as tmp:       # poses but neither map.ply nor frames: nothing to rebuild the cloud from
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        r = run(tmp, out, args=["--reuse-slam"])
        assert r.returncode != 0 and "step 'reuse'" in r.stderr, r.stderr


def test_without_reuse_slam_an_old_run_is_replayed():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "o"
        (out / "vs" / "run").mkdir(parents=True)
        (out / "vs" / "run" / "alidarState.txt").write_text("1 0 0 0 0 0 0 1\n")
        assert run(tmp, out).returncode == 0 and "run" in tools_logged(tmp)


def test_the_warning_about_lost_scans_reaches_the_user_and_does_not_fail_the_run():
    with tempfile.TemporaryDirectory() as tmp:
        r = run(tmp, Path(tmp) / "o", {"STUB_WARN": "1"})
        assert r.returncode == 0 and "WARNING: only 90 of 100" in r.stderr and "PLAY_RATE=1" in r.stderr, (r.returncode, r.stderr)
        assert "WARNING" not in run(tmp, Path(tmp) / "o2").stderr


def test_the_default_engine_check_and_make_map_are_the_scripts_next_to_it():
    """make_map.sh copied next to stand-ins for ghosts.py and make_map.py, with ENGINE_CHECK and MAKE_MAP_PY unset: it runs its own defaults."""
    with tempfile.TemporaryDirectory() as tmp:
        copy = Path(tmp) / "copy"
        copy.mkdir()
        (copy / "make_map.sh").write_text(SCRIPT.read_text())
        (copy / "ghosts.py").write_text('import os, sys\nopen(os.environ["LOG"], "a").write("ghosts_py " + " ".join(sys.argv[1:]) + "\\n")\n')
        (copy / "make_map.py").write_text('import os, sys\nopen(os.environ["LOG"], "a").write("make_map_py " + " ".join(sys.argv[1:]) + "\\n")\nout = sys.argv[sys.argv.index("--out") + 1]\n'
                                          'for f in ("map.ply", "map_clean.ply", "map.pgm", "map.yaml"): open(os.path.join(out, f), "w").close()\nopen(os.path.join(out, "report.json"), "w").write("{}")\n')
        tools = Path(tmp) / "tools"
        tools.mkdir()
        (tools / "run.sh").write_text("#!/bin/bash\n" + STUBS["run.sh"])
        (tools / "run.sh").chmod(0o755)
        env = {**{k: v for k, v in os.environ.items() if k not in ("ENGINE_CHECK", "MAKE_MAP_PY", "PLAY_RATE")}, "LOG": str(Path(tmp) / "log.txt"), "VOXELSLAM_RUN": str(tools / "run.sh")}
        r = subprocess.run(["bash", str(copy / "make_map.sh"), "bag dir", str(Path(tmp) / "o")], env=env, capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        assert [l.split()[0] for l in log(tmp).splitlines()] == ["ghosts_py", "run", "make_map_py"], log(tmp)
        assert log(tmp).splitlines()[0] == "ghosts_py --check-engine"


def test_the_flags_make_map_passes_exist_in_the_real_tool():
    helptext = subprocess.run([sys.executable, str(HERE / "make_map.py"), "--help"], capture_output=True, text=True).stdout
    for flag in ("--run", "--cloud", "--poses", "--bag", "--out", "--res", "--voxel", "--ceil", "--topic", "--play-rate"):
        assert flag in helptext, (flag, helptext)
    assert "--check-engine" in (HERE / "ghosts.py").read_text()
