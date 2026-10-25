"""Run: python3 -B test_make_cross.py"""
import make_cross as mc


def test_trigger_makes_the_robot_arrive_at_the_crossing_when_the_walker_reaches_the_lane():
    # walker needs 5 m / 1 m/s = 5 s to reach the lane centre; robot at 0.6 m/s must need exactly 5 s from the point where it releases the walker
    x = mc.trigger_x(cross_x=11.0, walker_s=5.0, walker_v=1.0, robot_v=0.6, delta=0.0)
    assert abs((11.0 - x) / 0.6 - 5.0) < 1e-9 and abs(x - 8.0) < 1e-9


def test_positive_delta_means_the_robot_arrives_later_than_the_walker():
    early = mc.trigger_x(11.0, 5.0, 1.0, 0.6, delta=-1.0)     # robot 1 s before the walker: released later (closer to the crossing)
    late = mc.trigger_x(11.0, 5.0, 1.0, 0.6, delta=+1.0)
    assert late < 8.0 < early and abs((early - late) - 1.2) < 1e-9


def test_build_has_both_sides_and_the_trigger_circle_starts_at_the_computed_point():
    trials = mc.build(cross_x=11.0, half=5.0, walker_v=1.0, robot_v=0.6, deltas=[0.0], start=[2.0, 0.0, 0.0], goal=[22.0, 0.0, 0.0], radius=0.3)
    assert [t["id"] for t in trials] == ["cross_n_d+0.0", "cross_s_d+0.0"]
    n, s = trials[0]["dyn"][0], trials[1]["dyn"][0]
    assert n["path"] == [[11.0, 5.0], [11.0, -5.0]] and s["path"] == [[11.0, -5.0], [11.0, 5.0]]
    assert abs(n["trigger"][0] - (8.0 + 0.3)) < 1e-9 and n["trigger"][2] == 0.3     # the circle is entered at x = 8.0: the walker is released there
    assert n["model"] == "b_walker" and n["speed"] == 1.0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
