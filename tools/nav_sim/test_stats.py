"""Run: python3 -B test_stats.py"""
import numpy as np

import stats as s


def test_wilson_reference_values():
    lo, hi = s.wilson(7, 10)
    assert abs(lo - 0.3968) < 2e-3 and abs(hi - 0.8922) < 2e-3
    lo, hi = s.wilson(0, 10)
    assert lo == 0.0 and abs(hi - 0.2775) < 2e-3
    lo, hi = s.wilson(10, 10)
    assert hi == 1.0 and abs(lo - 0.7225) < 2e-3


def test_wilson_narrows_with_n():
    w10 = np.subtract(*s.wilson(7, 10)[::-1])
    w100 = np.subtract(*s.wilson(70, 100)[::-1])
    assert w100 < w10 / 2


def test_paired_bootstrap_finds_a_real_shift_and_no_shift():
    rng = np.random.default_rng(1)
    base = rng.normal(10.0, 2.0, 40)
    est, lo, hi = s.paired_bootstrap_ci(base, base + 1.0 + rng.normal(0, 0.3, 40), seed=0)
    assert lo > 0.5 and hi < 1.5 and lo < est < hi
    est0, lo0, hi0 = s.paired_bootstrap_ci(base, base, seed=0)
    assert est0 == lo0 == hi0 == 0.0
    # pure noise with the same mean: the interval must contain 0
    _, lo1, hi1 = s.paired_bootstrap_ci(base, base + rng.normal(0, 1.0, 40) - 0.0, seed=0)
    assert lo1 < 0 < hi1


def test_paired_bootstrap_is_reproducible_and_pairs_by_index():
    a, b = np.array([1.0, 2.0, 3.0, 4.0]), np.array([2.0, 2.5, 3.0, 9.0])
    assert s.paired_bootstrap_ci(a, b, seed=3) == s.paired_bootstrap_ci(a, b, seed=3)
    est, _, _ = s.paired_bootstrap_ci(a, b, seed=3, stat=np.median)
    assert est == np.median(b - a)
    try:
        s.paired_bootstrap_ci(a, b[:3])
    except ValueError:
        return
    raise AssertionError("unequal lengths must raise")


def test_mcnemar_exact():
    assert abs(s.mcnemar_exact(0, 5) - 0.0625) < 1e-12
    assert s.mcnemar_exact(3, 3) == 1.0
    assert s.mcnemar_exact(0, 0) == 1.0
    assert abs(s.mcnemar_exact(5, 0) - 0.0625) < 1e-12      # symmetric


def test_paired_success_counts():
    base = [True, True, False, False, True]
    cand = [True, False, True, True, True]
    assert s.discordant(base, cand) == (1, 2)               # base only succeeded in 1 pair, candidate only in 2


def test_holm_adjusted():
    assert np.allclose(s.holm([0.01, 0.04, 0.03]), [0.03, 0.06, 0.06])
    assert np.allclose(s.holm([0.5]), [0.5])
    assert max(s.holm([0.9, 0.8])) <= 1.0


def test_summary():
    r = s.summary([1.0, 2.0, 3.0, 4.0, 100.0])
    assert r["n"] == 5 and r["median"] == 3.0 and r["max"] == 100.0 and r["min"] == 1.0
    assert s.summary([])["n"] == 0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
