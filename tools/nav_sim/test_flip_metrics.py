"""Run: python3 -B test_flip_metrics.py"""
import flip_metrics as fm

DOOR = (11.5, 12.5, -1.0, 1.0)      # x0, x1, y0, y1
GAP = (11.5, 12.5, 4.5, 7.0)


def plan(through):
    return [[2.0, 0.0], [8.0, 0.0], [12.0, 0.0 if through == "door" else 6.0], [22.0, 0.0]]


def test_a_plan_is_classified_by_the_region_it_crosses():
    assert fm.classify(plan("door"), {"door": DOOR, "gap": GAP}) == "door"
    assert fm.classify(plan("gap"), {"door": DOOR, "gap": GAP}) == "gap"
    assert fm.classify([[2.0, 0.0], [5.0, 0.0]], {"door": DOOR, "gap": GAP}) == "none"            # a plan that ends before the wall


def test_flips_count_changes_between_the_two_routes_and_ignore_plans_that_cross_neither():
    seq = ["door", "door", "gap", "gap", "door", "none", "door"]
    assert fm.flips(seq) == 2                                                                      # door->gap, gap->door; the "none" plan is skipped


def test_no_flip_when_the_route_never_changes_or_is_never_known():
    assert fm.flips(["door"] * 5) == 0 and fm.flips(["none", "none"]) == 0 and fm.flips([]) == 0


def test_trial_summary_reads_plan_all_and_reports_flips_and_final_route():
    rec = {"id": "x", "status": "succeeded", "duration": 31.0, "recoveries": 0, "plan_all": [[1.0, plan("door")], [2.0, plan("gap")], [3.0, plan("door")], [4.0, plan("door")]]}
    r = fm.trial_summary(rec, {"door": DOOR, "gap": GAP})
    assert r["flips"] == 2 and r["first"] == "door" and r["last"] == "door" and r["n_plans"] == 4 and r["duration"] == 31.0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
