"""Run: python3 -B test_make_mission.py"""
import math

import make_mission as mm


def trial(i, start, goal):
    return {"id": f"p{i:02d}", "start": [*start, 0.0], "goal": [*goal, 0.0], "euclid": 1.0, "shortest": 1.0}


def test_each_mission_starts_at_the_first_start_passes_through_the_goals_in_between_and_ends_at_the_last_goal():
    ts = [trial(0, (0, 0), (3, 0)), trial(1, (9, 9), (3, 4)), trial(2, (8, 8), (0, 4)), trial(3, (7, 7), (1, 1))]
    out = mm.build(ts, 2)
    assert [m["id"] for m in out] == ["m00", "m01"]
    assert out[0]["start"] == [0, 0, 0.0] and out[0]["via"] == [[3, 0, 0.0]] and out[0]["goal"] == [3, 4, 0.0]
    assert out[1]["start"] == [8, 8, 0.0] and out[1]["via"] == [[0, 4, 0.0]] and out[1]["goal"] == [1, 1, 0.0]


def test_the_reference_length_is_the_straight_line_chain_through_the_stops():
    out = mm.build([trial(0, (0, 0), (3, 0)), trial(1, (9, 9), (3, 4))], 2)
    assert abs(out[0]["shortest"] - 7.0) < 1e-9          # 3 m to the via, 4 m to the goal


def test_an_incomplete_last_group_is_dropped():
    assert len(mm.build([trial(i, (0, 0), (i, 1)) for i in range(5)], 2)) == 2


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
