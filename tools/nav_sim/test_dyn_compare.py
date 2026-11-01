"""Run: python3 -B test_dyn_compare.py"""
import dyn_compare as dc


def m(i, reached, contact, gap, dur, wait=0.0, started=True, active=None):
    return {"id": i, "reached": reached, "contact": contact, "gap_body": gap, "duration": dur, "wait_s": wait, "started": started, "active_contact": contact if active is None else active}


def test_pairs_by_seed_and_trial_id_even_when_the_order_differs():
    base = [[m("a", True, False, 1.0, 30), m("b", False, True, 0.0, 60)]]
    cand = [[m("b", True, False, 0.4, 31), m("a", True, False, 1.0, 29)]]
    r = dc.compare(base, cand)
    assert r["n"] == 2 and r["reached"] == (0, 1) and r["contact"] == (0, 1)          # (only base, only candidate); for contact: pairs where only that side avoided it


def test_a_different_order_does_not_create_differences():
    base = [[m("a", True, False, 1.0, 30), m("b", False, True, 0.0, 60)]]
    cand = [[m("b", False, True, 0.0, 60), m("a", True, False, 1.0, 30)]]          # identical results, other order
    r = dc.compare(base, cand)
    assert r["reached"] == (0, 0) and r["contact"] == (0, 0) and r["gap_body"][0] == 0.0


def test_counts_are_direction_aware_for_contacts_lower_is_better():
    base = [[m("a", True, False, 1.0, 30), m("b", True, True, 0.0, 30)]]
    cand = [[m("a", True, True, 0.0, 30), m("b", True, False, 0.5, 30)]]
    r = dc.compare(base, cand)
    assert r["contact"] == (1, 1)                                                      # base avoided "a", candidate avoided "b"


def test_duration_only_over_pairs_reached_by_both():
    base = [[m("a", True, False, 1, 30), m("b", False, False, 1, 120)]]
    cand = [[m("a", True, False, 1, 33), m("b", True, False, 1, 30)]]
    r = dc.compare(base, cand)
    assert r["duration"][0] == 3.0 and r["n_duration"] == 1                            # "b" was not reached by the base: no duration pair


def test_seeds_are_separate_pairs():
    base = [[m("a", True, False, 1, 30)], [m("a", False, False, 1, 120)]]
    cand = [[m("a", True, False, 1, 30)], [m("a", True, False, 1, 30)]]
    r = dc.compare(base, cand)
    assert r["n"] == 2 and r["reached"] == (0, 1)


def test_pairs_where_either_obstacle_never_started_are_left_out_and_counted():
    base = [[m("a", True, False, 1.0, 30), m("b", True, False, 1.0, 30)]]
    cand = [[m("a", True, False, 1.0, 30), m("b", False, False, None, 120, started=False)]]
    r = dc.compare(base, cand)
    assert r["n"] == 1 and r["not_started"] == 1 and r["reached"] == (0, 0)


def test_a_contact_with_the_robot_already_stopped_does_not_count_against_it():
    base = [[m("a", True, True, 0.0, 30), m("b", True, True, 0.0, 30)]]
    cand = [[m("a", True, True, 0.0, 30, active=False), m("b", True, True, 0.0, 30)]]     # in "a" the candidate was standing still when it was hit
    r = dc.compare(base, cand)
    assert r["contact"] == (0, 1) and r["totals"]["contact"] == (2, 1), r


def test_a_missing_trial_is_refused_not_silently_dropped():
    try:
        dc.compare([[m("a", True, False, 1, 30)]], [[m("b", True, False, 1, 30)]])
    except ValueError as e:
        assert "a" in str(e) or "b" in str(e)
        return
    raise AssertionError("must refuse")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
