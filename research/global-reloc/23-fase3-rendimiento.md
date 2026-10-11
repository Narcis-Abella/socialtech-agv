# Note 23 - Phase 3: performance and resources on the Orin NX (candidate GR-S2S3-F2c, frozen)

Status: closed, see section 10 (R-REV-SW = 2.5 s adopted as an internal software requirement).

Raw data on the NX: `/home/jetson/work/reloc-f3-20261011/` (14 new runs, `ev/` = evidence copy) and the 99 runs of `/home/jetson/work/reloc-f2l-20261011/runs/` (final tree of note 22, re-analysed, nothing re-run). Evidence in `results/23-f3-performance/`. Labels: **M** measured, **I** inferred.

## 1. Scope and what was NOT run
- Reused (no new runs): per-stage times, per-process cost (`proc_cost.py`) and `tegrastats` of the 99 runs of the F2c tree (1x replay, `jetson_clocks` active, idle board). New: `f3_summary.py` (aggregator, reads only).
- New runs (14, one at a time, idle board, 1x): 6 nominal repeats (`b1`-`b3`, eco01 z03 own = S2, hospital z00 own = S3), 4 with a container CPU quota (`--cpus 4`, `--cpus 2`; `DOCKER_CPUS` knob in `run_online.sh`, default 8, harness only), 4 under `--cpus 2` with the supervision (1 no-fault control, 3 x `k11` gyro fault, the F2 reference case).
- **Not run**: Orin Nano 8 GB (no access in this session; excluded, not measured - the `--cpus 2` runs are a proxy of CPU shortage, not a Nano result); cold boot clock A (BLOCKED_PHYSICAL); 10 repeats of two windows (3 per window run + 32 identical-input c04 runs of the F2 tree, see 4); Nav2 planner/controller running next to the stack (not part of the ensayada stack); disk stress (forbidden).
- Method files identical to note 22 (`tools-sha256-f3.txt` vs `f2l/tools-sha256-f2l.txt`: only `run_online.sh` knob and `f3_summary.py` differ).

## 2. Resources (M, 99 runs of F2c tree, 14 new runs agree)
| | median | p95 | max |
|---|---|---|---|
| System RAM used (tegrastats) | 2095 MB | 2138 | 2200 |
| Swap | 73 MB constant, rise 0 in all 99 | | |
| GPU GR3D | 0 % in all samples (no CUDA process) | | |
| Tj / CPU / GPU temperature, max per run | 66.6 / 66.3 / 62.8 C | 67.1 / 66.8 / 63.1 | 67.2 / 67.2 / 63.3 C |
| VDD_IN mean / peak per run | 9.9 / 13.9 W | 10.1 / 13.9 | 10.2 / 14.05 W |
| Cores busy (of 8), mean / p95 of samples | 1.34 / 2.74 | 1.63 / 3.00 | 1.78 / 3.08 |

Per process (cost sampler 2 Hz; CPU = mean over the whole run, 100 % = one core; PSS peak):
| process | CPU mean median (max) | PSS peak median (max) |
|---|---|---|
| `search_v2` (S2/S3 search, while searching) | 103 % (115) | 62 MB (64) |
| `reloc_node` (state machine + supervision) | 31 % (38) | 101 MB (288; highest in the S3 hospital runs) |
| `register_v2` (GICP worker, one per check) | 4.7 % (12.7) | 52 MB (54) |
| FAST-LIO2 | 17 % (22) | 39 MB (76) |
| AMCL / scan conv / map_server / lifecycle | 9 / 1.9 / 1.3 / 1.5 % | 16 / 15 / 13 / 9 MB |
Memory trend of a 492 s LOCALIZED run (note 22): node +0.22 MB/min, FAST-LIO2 +0.98 MB/min, AMCL +0.05 MB/min (slopes over 8 min, not a proof of no leak beyond that time).

## 3. Timing (M, accepted attempts; protocol clock = first sensor message)
| Window | accept | LOCALIZED | FFT primary / confirm | GICP | search worker | map check |
|---|---|---|---|---|---|---|
| S2 eco01 z03 own, N=3 | 18.80-18.95 s | 25.09-25.15 s | 2.90 / 2.82 s | 3.2-3.3 s | 9.8 s | 0.75 s |
| S3 hospital z00 own, N=3 | 41.81-41.96 s | 48.41 s | 3.15 / 3.29 s | 3.6 s | 11.1 s | 0.8 s |
Margin to the 60 s deadline: S2 41 s, S3 18 s of acceptance (S3 includes the 3 s exposure and the supervised S3 accumulation). All 18 clean attempts of the F2c tree: accept median 18.9 s, p95 36.8 s, max 42.0 s (mixed S2/S3, `stages-clean.json`).
Supervision (note 22, 99 + 14 runs): one check every ~0.55 s, 0.35 s per check (median), last-pass age <= 1.17 s without faults; revocation after a fault 0-1.46 s (note 22).

## 4. Noise floor (M)
- S2 window c04, identical input, 32 runs of the F2 tree (faults enter only after acceptance): accept 18.94-19.23 s, median 19.06, **spread 0.29 s**.
- Three new repeats per window: S2 eco01 z03 spread 0.15 s; S3 hospital z00 spread 0.15 s, same pose (error 0.066 m / 0.25 deg in the wide bracket, within limits). Two of the three S3 runs (`b1`, `b3`) are classed `unscorable_reference_gap` by the evaluator (reference not bracketed within 0.2 s at the query time); the wide-bracket error is identical in both. This is a scoring limitation, not a method difference.
- Differences below ~0.3 s are inside the noise; the 2.1-2.4 s of 5 are not.

## 5. Concurrency: CPU quota (M)
| quota | S3 hospital accept / LOCALIZED | S2 eco01 accept / LOCALIZED | FFT primary |
|---|---|---|---|
| 8 (reference) | 41.81-41.96 / 48.41 s | 18.80-18.95 / 25.1 s | 3.15 / 2.90 s |
| 4 | 41.85 / 48.40 | 18.83 / 25.08 | 3.14 / 2.91 |
| 2 | **44.25 / 50.70** | **21.02 / 27.47** | **4.31 / 4.00** |
`--cpus 4`: no change. `--cpus 2` (a CFS quota; the container still sees 8 cores): FFT +1.2 s per pass, search +2.1-2.2 s, accept +2.1-2.4 s, GICP unchanged, outcome and pose unchanged, margin to 60 s still 15.8 s (S3). Under `--cpus 2` after LOCALIZED: no-fault control `k00`: 89 checks, 0 failed, permit never withheld, last-pass age max 1.09 s, worker 58 % of a core; `k11` x3: revocation latency **0.65, 0.15, 0.01 s** (F2 tree at quota 8: 0, 0.86, 0 s), all `LOCALIZATION_LOST(map_incoherent)`, gate closed each time. The freshness rule (2.0 s) never tripped spuriously. Not covered: other loads (Nav2, perception) beyond this quota proxy.

## 6. NVMe (M for counts, I for causes)
- Kernel journal (`journalctl -k`, no root): 145 `nvme0 ... I/O timeout, completion polled` lines since Oct 5; by day 1 / 4 / 3 / 27 / 59 / 51 (Oct 5, 7, 8, 9, 10, 11). Rate rose in the days of the long campaigns; I cannot say whether it is load, firmware, power state or media.
- `io_stall_audit.py` over 113 runs (F2c tree + F3): **8 overlapped a timeout line; one (h25, 26.9 s) had a hole in the 2 Hz sampler and was already invalidated and repeated (note 22). The other 7 ended in the expected state with sampler gap 0.5 s.** None of the 14 new runs overlapped a timeout.
- Drive temperature (hwmon) 39.9 C idle, disk 46 % used. SMART not readable without root: `sudo nvme smart-log /dev/nvme0` (media errors, percentage used, unsafe shutdowns) is still the missing evidence; kernel parameters (APST) are not touched.
- Interference with the algorithm: none observed. A stall of >2 s in the bag player starves FAST-LIO2 and the node revokes `odometry_stalled` (h25 first run), which is the intended fail-safe, not an algorithm change.

## 7. Anomalies and decisions
- `unscorable_reference_gap` x2 (see 4): not a regression, wide-bracket error recorded.
- Peak PSS of `reloc_node` 288 MB (highest in the S3 hospital runs, I), against 101 MB median: transient, inside the 16 GB board (system max 2.2 GB).
- No optimisation done: resources are low (1.3 of 8 cores, 0 % GPU, 2.2 GB, 67 C) and timing has margin; no functional change, F2c frozen.
- Residual CPU is dominated by the 0.5 s supervision (+17.5 points of a core, note 22) and the one-core searches.

## 8. Criteria
| Criterion | |
|---|---|
| Resource use compatible with the stack tried | met (M): max 2.2 GB RAM, 0 % GPU, 67 C, 14 W peak, <=3.1 cores p95 |
| Timing acceptable, no critical F2 regression | met (M): 41 s / 18 s margin; cpus 2 +2.4 s; revocation 0-0.65 s at cpus 2, 0-1.46 s at 8 |
| Concurrency stable | met for the stack, quota 8/4/2 (M); Nav2/AI layer next to it not measured |
| No relevant degradation | met: swap constant, memory slopes small, no orphan processes (`orphans` 0 in all) |
| Reproducible, traceable | met: tables and commands below |
| NVMe evaluated, limits documented | met as far as it goes without root (6); SMART pending |
Verdict: **FASE 3: VERIFIED - VALIDATED_IN_REPLAY on the Orin NX**, restricted to the stack ensayada and replay at 1x. Excluded explicitly: Orin Nano, cold boot clock A (BLOCKED_PHYSICAL), Nav2 concurrency, physical validation. R-REV = 2.5 s remains a provisional software criterion pending Narcis.

## 9. Reproduce
`f3_summary.py stages|teg|cost RUNS_DIR [REGEX]` (aggregates), `io_stall_audit.py RUNS_DIR...`; runs: `chain9.sh` (`batch_s2s3.sh cases-f3.csv TAG`, `DOCKER_CPUS=N`, `batch_latency.sh f3q2 k00`, `... f3q2r k11_gyro_0p5_from_35`) in a new NX root copied from `reloc-f2l-20261011` (`tools`, `windows`). Outputs: `results/23-f3-performance/` (`f3/` new runs, `f2c-runs/` re-analysis of the 99 runs).

## 10. Closure (2026-10-11, documentation only)
- Decision of the team after reviewing this note: **F3 = VERIFIED - VALIDATED_IN_REPLAY_ON_JETSON_NX**. The evidence is sufficient for the scope evaluated; no more benchmarks or optimisation in this phase. F2c stays frozen; no code or evidence changed by this closure.
- **R-REV-SW = 2.5 s** (from note 22) is adopted as an **internal software requirement** for revocation of the authorization (fault after `LOCALIZED` -> zero gate output), applicable to the replay scenarios and conditions of notes 22 and 23 (measured max 1.46 s at quota 8, 0.65 s at `--cpus 2`). It is **not** a physical safety limit and not a universal guarantee. Real stop, base firmware command timeout, physical CAN and motion remain pending for Narcis; the 0.5-0.85 m of blind travel in note 22 still needs his judgement.
- Figures re-checked against `results/23-f3-performance/` (`f2c-runs/teg-all.json`: 99 runs, RAM max 2200 MB, swap rise 0, GR3D 0 %, Tj max 67.218 C; `f3/teg-f3.json`: 14 runs, RAM max 2158 MB, Tj max 66.812 C). The tables above were not changed.
- Phase status: F1 VERIFIED; F2 VERIFIED (`VALIDATED_IN_REPLAY`); F3 VERIFIED (`VALIDATED_IN_REPLAY_ON_JETSON_NX`); F4, F5, F6 BLOCKED_PHYSICAL; F7 PENDING.
