"""Comparison statistics for paired navigation experiments (numpy only). The same scenarios are run with a base and a candidate, so differences are taken per scenario."""
import math

import numpy as np


def wilson(k, n, z=1.96):
    """Wilson score interval for k successes in n trials."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def paired_bootstrap_ci(base, cand, n_boot=10000, seed=0, stat=np.mean, alpha=0.05):
    """(stat of cand - base, lo, hi): percentile bootstrap over scenarios of the per-scenario difference."""
    base, cand = np.asarray(base, float), np.asarray(cand, float)
    if base.shape != cand.shape:
        raise ValueError("base and candidate must have one value per scenario")
    d = cand - base
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(d), size=(n_boot, len(d)))
    boots = np.array([stat(d[i]) for i in idx])
    return (float(stat(d)), float(np.percentile(boots, 100 * alpha / 2)), float(np.percentile(boots, 100 * (1 - alpha / 2))))


def discordant(base_ok, cand_ok):
    """(pairs only the base solved, pairs only the candidate solved)."""
    b, c = np.asarray(base_ok, bool), np.asarray(cand_ok, bool)
    return int(np.sum(b & ~c)), int(np.sum(~b & c))


def mcnemar_exact(b, c):
    """Two-sided exact p-value for the discordant pair counts (b: only base, c: only candidate)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def holm(pvals):
    """Holm-Bonferroni adjusted p-values, in the input order."""
    p = np.asarray(pvals, float)
    order = np.argsort(p)
    adj = np.empty_like(p)
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, (len(p) - rank) * p[i])
        adj[i] = min(1.0, running)
    return adj


def summary(x):
    x = np.asarray(x, float)
    if not x.size:
        return {"n": 0}
    return {"n": int(x.size), "median": float(np.median(x)), "mean": float(np.mean(x)), "p95": float(np.percentile(x, 95)), "min": float(x.min()), "max": float(x.max())}
