"""Run: python3 -B test_legend.py"""
import legend


def test_one_line_per_part_then_the_trial():
    assert legend.legend_lines("f_lat | overlays: fast lat | set: x=1", "head0", 2, 5) == ["f_lat", "overlays: fast lat", "set: x=1", "trial head0  (2/5)"]


def test_without_a_legend_the_trial_is_still_shown():
    assert legend.legend_lines("", "block", 1, 5) == ["(no legend set)", "trial block  (1/5)"]
    assert legend.legend_lines(None, "block", 1, 5)[-1] == "trial block  (1/5)"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
