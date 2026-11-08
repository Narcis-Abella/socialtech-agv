"""Numbers that are written in several files and must be the same one. Each is a copy because the files run in different places (host, container, laptop, sim);
this reads them all and fails when one moves alone. Run: python3 ops/test_constants_agree.py   (no ROS needed; PyYAML)"""
import ast
import os
import re
import sys

import yaml

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def text(path):
    return open(os.path.join(REPO, path)).read()


def find(path, pattern, group=1, flags=0):
    m = re.search(pattern, text(path), flags)
    assert m, f"{path}: {pattern!r} not found"
    return m.group(group)


def test_the_obstacle_band_is_the_same_for_the_scan_the_costmap_and_the_map():
    scan = yaml.safe_load(text("src/bringup/socialtech_bringup/config/scan.yaml"))
    p = next(v for k, v in scan.items() if isinstance(v, dict)).get("ros__parameters", scan)
    lo, hi = p["min_height"], p["max_height"]
    assert (lo, hi) == (0.10, 1.80), (lo, hi)
    lo_f, hi_f = (float(v) for v in re.search(r"OBST_LO, OBST_HI = ([\d.]+), ([\d.]+)", text("tools/slam_bench/floor_level.py")).groups())
    assert (lo_f, hi_f) == (lo, hi), "floor_level.OBST_LO/HI (the band the map is rasterized with) vs scan.yaml"
    assert float(find("tools/slam_bench/make_map.sh", r'"\$\{CEIL:=([\d.]+)\}"')) == hi, "make_map.sh CEIL default vs the top of the band"
    gc = yaml.safe_load(text("src/navigation/socialtech_nav/config/nav2_params.yaml"))["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    assert (gc["min_obstacle_height"], gc["max_obstacle_height"]) == (lo, hi), "the global costmap's marking cut vs the band"


def test_the_lidar_offset_is_fast_lios_extrinsic_everywhere_it_is_copied():
    launch = ast.literal_eval(find("src/bringup/socialtech_bringup/launch/nav_live.launch.py", r"^LIDAR_IN_BODY = (\[[^\]]*\])", flags=re.M))
    ghosts = ast.literal_eval(find("tools/slam_bench/ghosts.py", r"^EXTR = (\([^)]*\))", flags=re.M))
    frames = ast.literal_eval(find("src/localization/gn_supervisor/scripts/livox_frames.py", r"extrinsic=(\([^)]*\))"))
    assert list(ghosts) == launch and list(frames) == launch, (launch, ghosts, frames)


def test_the_livox_addresses_and_the_receive_buffer_have_one_value():
    for path in ("src/bringup/socialtech_bringup/scripts/livox_config.sh", "ops/nav_live.sh", "ops/record_bag.sh"):
        assert find(path, r"^HOST_IP=\$\{LIVOX_HOST_IP:-([\d.]+)\}", flags=re.M) == "192.168.1.50", path
        assert find(path, r"^LIDAR_IP=\$\{LIVOX_LIDAR_IP:-([\d.]+)\}", flags=re.M) == "192.168.1.126", path
    assert find("host/robot.conf.example", r"^LIVOX_HOST_IP=([\d.]+)", flags=re.M) == "192.168.1.50"
    assert find("host/setup_host.sh", r"\$\{LIVOX_HOST_IP=([\d.]+)\}") == "192.168.1.50"
    rmem = {find(p, r"^(?:RMEM_MAX|RMEM_WANT)=(\d+)", flags=re.M) for p in ("host/setup_host.sh", "ops/nav_live.sh", "ops/record_bag.sh")}
    assert rmem == {"67108864"}, rmem


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    sys.exit(0)
