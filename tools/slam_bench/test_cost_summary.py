"""Checks cost_summary.py on real tegrastats lines of the three Jetsons. Run: python3 test_cost_summary.py"""
import cost_summary as cs

AGX = ("10-01-2026 11:05:58 RAM 8207/62878MB (lfb 80x4MB) SWAP 85/6144MB (cached 0MB) CPU [9%@729,8%@729,9%@729,7%@729,3%@1190,19%@1190,3%@1190,57%@1190,6%@729,5%@729,1%@729,1%@729] "
       "GR3D_FREQ 0% cpu@47.906C/48.343C soc2@45.375C/45.406C soc0@45.5C/45.593C tj@47.906C/48.343C soc1@47.093C/47.25C VDD_GPU_SOC 3034mW/3049mW/3412mW VDD_CPU_CV 1137mW/1296mW/2273mW VIN_SYS_5V0 4939mW/4924mW/5040mW")
NANO = ("10-01-2026 20:36:32 RAM 974/7546MB (lfb 6x4MB) SWAP 0/5821MB (cached 0MB) CPU [0%@729,0%@729,0%@883,0%@883,0%@729,0%@729] GR3D_FREQ 12% cpu@49.843C/49.843C soc2@49.156C/49.156C "
        "soc0@50.187C/50.187C gpu@50.875C/50.875C tj@50.875C/50.875C soc1@50.218C/50.218C VDD_IN 4548mW/4548mW/4548mW VDD_CPU_GPU_CV 483mW/483mW/483mW VDD_SOC 1489mW/1489mW/1489mW")
NX = ("10-01-2026 20:36:35 RAM 2432/15598MB (lfb 4x4MB) SWAP 0/6144MB (cached 0MB) CPU [55%@1036,9%@1036,24%@1036,64%@1036,off,off,0%@729,0%@729] GR3D_FREQ 0% cv0@56.281C/56.281C "
      "cpu@57.156C/57.156C soc2@56.593C/56.593C soc0@57.781C/57.781C cv1@56.125C/56.125C gpu@55.343C/55.343C tj@59.687C/59.687C soc1@59.687C/59.687C cv2@56.281C/56.281C VDD_IN 7280mW/7280mW/7280mW")


def test_the_three_tegrastats_formats_are_parsed():
    a, n, x = (cs.parse_tegrastats([l])[0] for l in (AGX, NANO, NX))
    assert (a["ram"], a["ram_total"], len(a["cores"]), a["gpu"], a["tj"], a["vdd_in"]) == (8207, 62878, 12, 0, 47.906, None)    # the AGX has no total input power
    assert (n["ram"], n["ram_total"], len(n["cores"]), n["gpu"], n["tj"], n["vdd_in"]) == (974, 7546, 6, 12, 50.875, 4.548)
    assert (x["ram"], x["ram_total"], len(x["cores"]), x["tj"], x["vdd_in"]) == (2432, 15598, 8, 59.687, 7.28)
    assert x["cores"][4] == 0 and x["cores"][3] == 64                                                                           # an offline core counts as idle


def test_cores_used_is_the_sum_of_the_core_utilisations():
    assert abs(cs.parse_tegrastats([NX])[0]["used"] - (55 + 9 + 24 + 64) / 100) < 1e-9


def test_a_line_that_is_not_tegrastats_is_skipped():
    assert cs.parse_tegrastats(["", "garbage", NANO, "RAM only"]) == cs.parse_tegrastats([NANO])


def test_docker_stats_lines_give_cores_and_mib():
    s = cs.parse_dstats(["1790779514 77.68% 31.98MiB / 61.4GiB", "1790779520 110.89% 1.5GiB / 7.3GiB", "1790779526 ", "1790779532 5% 12.5kB / 7GiB"])
    assert len(s) == 2 and all(abs(x - y) < 1e-9 for got, want in zip(s, [(0.7768, 31.98), (1.1089, 1536.0)]) for x, y in zip(got, want))                                                                              # empty or non-MiB/GiB lines are skipped


def test_summary_reports_mean_p95_and_max():
    lines = [NANO.replace("CPU [0%@729,0%@729,0%@883,0%@883,0%@729,0%@729]", f"CPU [{c}%@729,0%@729,0%@883,0%@883,0%@729,0%@729]") for c in (100, 100, 200 // 2, 0)]
    r = cs.summarize(cs.parse_tegrastats(lines))
    assert r["seconds"] == 4 and abs(r["cores_mean"] - 0.75) < 1e-9 and r["cores_max"] == 1.0 and r["ram_max"] == 974 and abs(r["power_mean"] - 4.548) < 1e-9


if __name__ == "__main__":
    test_the_three_tegrastats_formats_are_parsed()
    test_cores_used_is_the_sum_of_the_core_utilisations()
    test_a_line_that_is_not_tegrastats_is_skipped()
    test_docker_stats_lines_give_cores_and_mib()
    test_summary_reports_mean_p95_and_max()
    print("ok")
