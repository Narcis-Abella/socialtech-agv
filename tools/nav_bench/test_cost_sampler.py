"""Checks cost_sampler.py parsing and summary (no /proc needed). Run: python3 test_cost_sampler.py"""
import cost_sampler as cs

# a real-looking /proc/<pid>/stat line: the command name may contain spaces and parentheses; utime = field 14, stime = field 15
STAT = "4242 (fast lio (map)) S 1 4242 4242 0 -1 4194560 100 0 0 0 1500 250 0 0 20 0 8 0 12345 123456789 5000 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 3 0 0 0 0 0"
STATUS = "Name:\tfastlio_mapping\nVmPeak:\t  900000 kB\nVmRSS:\t  234567 kB\nThreads:\t8\n"


def test_parse_stat_returns_cpu_ticks_even_with_odd_command_names():
    assert cs.cpu_ticks(STAT) == 1750


def test_parse_status_returns_rss_in_mb():
    assert abs(cs.rss_mb(STATUS) - 234567 / 1024) < 1e-9


def test_summary_gives_mean_p95_cpu_and_peak_rss_per_process_and_the_total():
    rows = [(float(t), "amcl", 30.0 + t, 100.0 + t) for t in range(0, 11)] + [(float(t), "fastlio_mapping", 100.0, 300.0) for t in range(0, 11)]
    s = cs.summarize(rows)
    assert abs(s["amcl"]["cpu_mean"] - 35.0) < 1e-9 and s["amcl"]["cpu_p95"] == 39.5 and s["amcl"]["rss_peak_mb"] == 110.0, s
    assert s["TOTAL"]["cpu_mean"] == 135.0 and s["TOTAL"]["rss_peak_mb"] == 410.0, s  # cpu in % of ONE core; total = sum per sample time


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
