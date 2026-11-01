"""map_report: report.json of a make_map run, from the run's poses, node.log, the bag metadata and the map files. The numbers of the floor vote come from the pipeline, not from a log."""
import json

import map_report

NODE_LOG = "a\nOdometry degradation detected - resetting system\nb\nresetting system\nInsufficient IMU\nInsufficient IMU\n"
METADATA = """rosbag2_bagfile_information:
  version: 9
  message_count: 100
  topics_with_message_count:
    - topic_metadata:
        name: /livox/imu
        type: sensor_msgs/msg/Imu
      message_count: 73228
    - topic_metadata:
        name: /livox/lidar
        type: livox_ros_driver2/msg/CustomMsg
      message_count: 3662
"""


def make_run(tmp, poses, node_log=NODE_LOG, pgm=b"\x00\xfe\xcd\xcd", yaml_extra="# T_map_world (row-major; p_map = T p_ply):\n"):
    (tmp / "vs/run").mkdir(parents=True)
    poses_file = tmp / "vs/run/alidarState.txt"
    poses_file.write_text("".join(f"{100 + i} {x} {y} {z} 0 0 0 1\n" for i, (x, y, z) in enumerate(poses)))
    (tmp / "node.log").write_text(node_log)
    (tmp / "map.yaml").write_text("image: map.pgm\nmode: trinary\nresolution: 0.05\n" + yaml_extra)
    (tmp / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + pgm)
    return poses_file, tmp / "node.log"


def test_report_counts_resets_poses_path_closure_imu_warnings_height_and_cells(tmp_path):
    poses, log = make_run(tmp_path, [(0, 0, 0), (3, 0, 0), (3, 4, 0)])
    r = map_report.write_report(tmp_path, poses_file=poses, node_log=log, extra={"floor_tilt_deg": 0.94, "sensor_height_m": 1.392})
    assert r == json.loads((tmp_path / "report.json").read_text())
    assert (r["resets"], r["poses"], r["path_m"], r["end_to_start_m"], r["insufficient_imu"]) == (2, 3, 7.0, 5.0, 2)
    assert (r["floor_tilt_deg"], r["sensor_height_m"]) == (0.94, 1.392)      # given by the pipeline, not parsed from a log
    assert r["map"] == {"resolution_m": 0.05, "size_cells": [2, 2], "occupied": 1, "free": 1, "unknown": 2, "has_T_map_world": True}


def test_a_run_with_one_pose_and_a_map_without_occupied_cells(tmp_path):
    poses, log = make_run(tmp_path, [(1, 2, 3)], node_log="", pgm=b"\xfe\xfe\xcd\xcd", yaml_extra="")
    r = map_report.write_report(tmp_path, poses_file=poses, node_log=log, extra={})
    assert (r["poses"], r["path_m"], r["end_to_start_m"], r["resets"]) == (1, 0.0, 0.0, 0)
    assert r["map"]["occupied"] == 0 and r["map"]["has_T_map_world"] is False


def test_a_missing_node_log_is_not_an_error(tmp_path):
    poses, _ = make_run(tmp_path, [(0, 0, 0), (1, 0, 0)])
    r = map_report.write_report(tmp_path, poses_file=poses, node_log=None, extra={})      # a map regenerated from an old folder has no node.log
    assert r["resets"] is None and r["insufficient_imu"] is None and r["poses"] == 2


def test_the_extra_fields_of_the_pipeline_end_up_in_the_report(tmp_path):
    poses, log = make_run(tmp_path, [(0, 0, 0), (1, 0, 0)])
    extra = {"pipeline": 2, "commit": "abc1234", "play_rate": 2.0, "scans_in_bag": 2, "ghost_band": [None, None], "rule": {"ghost_vox_m": 0.05}}
    r = map_report.write_report(tmp_path, poses_file=poses, node_log=log, extra=extra)
    assert all(r[k] == v for k, v in extra.items())


def test_scans_in_bag_reads_the_message_count_of_the_topic_from_metadata_yaml(tmp_path):
    (tmp_path / "metadata.yaml").write_text(METADATA)
    assert map_report.scans_in_bag(tmp_path, "/livox/lidar") == 3662
    assert map_report.scans_in_bag(tmp_path, "/livox/imu") == 73228
    assert map_report.scans_in_bag(tmp_path, "/nope") is None
    assert map_report.scans_in_bag(tmp_path / "missing", "/livox/lidar") is None
