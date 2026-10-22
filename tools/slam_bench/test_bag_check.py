"""Checks bag_check.py: gaps in a message stream and the exit code on a synthetic bag. Run: python3 test_bag_check.py"""
import tempfile
from pathlib import Path

import numpy as np

import bag_check as bc

try:
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    Writer = None


def test_a_clean_stream_has_no_gaps():
    r = bc.check_stream(np.arange(0, 20) * 0.1)
    assert r["n"] == 20 and r["gaps"] == [] and r["lost"] == 0 and abs(r["period"] - 0.1) < 1e-9


def test_a_hole_is_reported_with_its_start_length_and_the_messages_it_swallowed():
    t = np.r_[np.arange(0, 10) * 0.1, np.arange(20, 30) * 0.1]            # 0.9 s -> 2.0 s: 10 messages gone
    r = bc.check_stream(t)
    assert len(r["gaps"]) == 1 and abs(r["gaps"][0][0] - 0.9) < 1e-9 and abs(r["gaps"][0][1] - 1.1) < 1e-9
    assert r["lost"] == 10


def test_jitter_below_two_and_a_half_periods_is_not_a_gap():
    t = np.cumsum(np.r_[0, np.tile([0.1, 0.18, 0.1, 0.12], 10)])
    assert bc.check_stream(t)["gaps"] == []


def test_a_fast_stream_needs_at_least_50_ms_to_count_as_a_gap():
    t = np.arange(0, 400) * 0.005                                        # 200 Hz
    assert bc.check_stream(np.delete(t, range(100, 105)))["gaps"] == []    # 25 ms hole: scheduler noise
    r = bc.check_stream(np.delete(t, range(100, 130)))                     # 150 ms hole: 30 samples lost
    assert len(r["gaps"]) == 1 and r["lost"] == 30


def test_a_stream_with_fewer_than_two_messages_cannot_be_judged():
    assert bc.check_stream(np.array([])) ["n"] == 0
    assert bc.check_stream(np.array([1.0]))["n"] == 1


def test_a_few_short_gaps_are_a_warning_but_a_long_gap_or_many_lost_messages_fail():
    base = np.arange(0, 6000) * 0.1
    assert bc.verdict(bc.check_stream(base)) == "ok"
    assert bc.verdict(bc.check_stream(np.delete(base, [1000, 1001, 3000, 3001, 5000, 5001]))) == "warn"      # 3 holes of 0.3 s, as in our own good bags
    assert bc.verdict(bc.check_stream(np.delete(base, range(2000, 2012)))) == "FAIL"                         # one hole of 1.3 s
    assert bc.verdict(bc.check_stream(np.delete(base, np.r_[np.arange(100, 5900, 40), np.arange(101, 5901, 40)]))) == "FAIL"   # 145 holes of 0.3 s: 4.8 % lost
    assert bc.verdict(bc.check_stream(np.array([1.0]))) == "FAIL"


def write_bag(path, lidar, imu):
    ts = get_typestore(Stores.LATEST)
    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as w:
        for topic, times in (("/livox/lidar", lidar), ("/livox/imu", imu)):
            if len(times):
                c = w.add_connection(topic, "std_msgs/msg/String", typestore=ts)
                for t in times:
                    w.write(c, int(t * 1e9), ts.serialize_cdr(ts.types["std_msgs/msg/String"](data="x"), "std_msgs/msg/String"))


def test_main_exits_0_on_a_clean_bag_and_1_on_a_gap_or_a_missing_topic(capsys=None):
    if Writer is None:
        return                                                          # rosbags missing: skipped
    lidar, imu = np.arange(0, 1000) * 0.1, np.arange(0, 2000) * 0.005
    with tempfile.TemporaryDirectory() as tmp:
        write_bag(Path(tmp) / "ok", lidar, imu)
        write_bag(Path(tmp) / "hole", np.delete(lidar, range(40, 60)), imu)        # 2 s without lidar
        write_bag(Path(tmp) / "nolidar", [], imu)
        write_bag(Path(tmp) / "small", np.delete(lidar, [40, 41]), imu)             # 0.3 s without lidar: a warning only
        assert bc.main([str(Path(tmp) / "ok")]) == 0
        assert bc.main([str(Path(tmp) / "hole")]) == 1
        assert bc.main([str(Path(tmp) / "small")]) == 0
        assert bc.main([str(Path(tmp) / "nolidar")]) == 1
        assert bc.main([str(Path(tmp) / "hole"), "--lidar", "/nothing"]) == 1


if __name__ == "__main__":
    test_a_clean_stream_has_no_gaps()
    test_a_hole_is_reported_with_its_start_length_and_the_messages_it_swallowed()
    test_jitter_below_two_and_a_half_periods_is_not_a_gap()
    test_a_fast_stream_needs_at_least_50_ms_to_count_as_a_gap()
    test_a_stream_with_fewer_than_two_messages_cannot_be_judged()
    test_a_few_short_gaps_are_a_warning_but_a_long_gap_or_many_lost_messages_fail()
    test_main_exits_0_on_a_clean_bag_and_1_on_a_gap_or_a_missing_topic()
    print("ok")
