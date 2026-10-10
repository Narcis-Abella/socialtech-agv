# 19 — Closing and delivery plan for the global relocalization candidate

Date: 2026-10-10. Branch `feat/global-reloc`. Candidate: `GR-S2S3-F1`, commit `c026a7b` (FAST-LIO2 -> FFT correlative search -> small_gicp -> frozen gate -> `/initialpose` -> AMCL).
This file is the index of the remaining work. Each phase is closed only by its own criterion. Update the status column here, nowhere else.

Phase status: `PENDING`, `IN_PROGRESS`, `VERIFIED`, `BLOCKED_PHYSICAL` (needs a person, the robot or a measurement).
Evidence levels, never mixed:

| Level | Meaning | Reached today |
|---|---|---|
| `VALIDATED_IN_REPLAY` | node + AMCL on recorded bags, 1x, on the NX | yes, development windows only (notes 15, 16, 20) |
| `VALIDATED_ON_HARDWARE_STATIONARY` | live sensors, robot still, motors off | no |
| `VALIDATED_WITH_SUPERVISED_MOTION` | S3 with a person moving the chair | no |
| `FORMALLY_BENCHMARKED` | frozen candidate run once on independent Q-eval data, note 18 section 6 | no |

`LOCALIZED` is the node's own state (AMCL coherent with the propagated pose). It is never a substitute for an external accuracy check.

## Summary

| Phase | Status | Level it can deliver |
|---|---|---|
| F1 Audit and freeze | **VERIFIED** (2026-10-10) | `VALIDATED_IN_REPLAY` |
| F2 Robustness | PENDING | `VALIDATED_IN_REPLAY` |
| F3 Performance and timeout | PENDING | `VALIDATED_IN_REPLAY`; cold boot part BLOCKED_PHYSICAL |
| F4 Physical calibration | BLOCKED_PHYSICAL | precondition of H1 |
| F5 Independent benchmark | BLOCKED_PHYSICAL (data to record) | `FORMALLY_BENCHMARKED` |
| F6 H1 / H2 validation | BLOCKED_PHYSICAL | `VALIDATED_ON_HARDWARE_STATIONARY`, `VALIDATED_WITH_SUPERVISED_MOTION` |
| F7 Documentation and handover to Narcís | PENDING | - |

## F1 — Audit and freeze — VERIFIED

- Objective: audit the online node, check the reports against the raw results, reproduce S2 / S3 / negative, freeze a reproducible candidate.
- Done: code audit, 2 demonstrated bugs fixed with regressions (IMU-gate stall on stamp jumps; reference visible in the method container), 9 open non-critical findings listed, note-16 numbers recomputed (1 erratum), 3 cases reproduced on the NX, tests run on the laptop and in the image, identity recorded, 4 decisions written for Narcís.
- Evidence: [20-auditoria-fase1-congelacion.md](20-auditoria-fase1-congelacion.md), `results/19-f1-audit/`, NX `/home/jetson/work/reloc-f1-audit-20261010/`.
- Closing criterion (met): full audit, evidence checked, S2 / S3 / negative reproduced, tests run, candidate identified, decisions documented, no open bug that invalidates the candidate.
- Carried forward, not blocking: findings O1-O9 of note 20; the four decisions are documented but not yet taken by Narcís.

## F2 — Robustness — PENDING

- Objective: close or consciously accept the open findings and the untested fault cases, without touching the gate or the search.
- Pending (each: evidence first, then a regression, then a new candidate name if method code changes):
  1. O1: both sensor streams absent -> no terminal state. Decide a start-up timeout or an external watchdog; test it.
  2. O2: define and test what the navigation side reads to enable motion (`/global_reloc/state == LOCALIZED`); behaviour of the TF after `HANDOFF_FAILED`.
  3. O3: AMCL lost after `LOCALIZED` (replay: sensor blanked for 20 s after handoff). Record, then decide whether the node must supervise.
  4. O4: memory of a long-lived node after `LOCALIZED` (measure PSS over 30 min of replay); bound the buffers if it grows.
  5. Untested in note 16: reversed-order stamps, a stamp jump to epoch end to end (unit-tested only), FAST-LIO2 alive but diverged, symmetric places, both streams absent.
  6. FAST-LIO2 start-up hang (1 in about 100): try to reproduce with 200 cold starts on one window; record the rate with its Wilson bound; no fix without a cause.
  7. O7: decide whether the evaluator may score a stamp that coincides with a reference sample; if changed, re-score every table with the new evaluator and say so.
  8. A ROS-free test seam for `reloc_node.py` decision paths (tick with fake clock) if the fixes above touch it.
- Evidence to produce: fault table like `results/16-s2s3/table-faults.csv` on the new candidate, with `orphans` and `pose_published` per row.
- Closing criterion: every item above is fixed with a regression or accepted in writing with its reason; all note-16 fault cases re-run on the resulting candidate with 0 poses without `ACCEPTED` and 0 orphans; S2 / S3 / negative reproduced again.
- Links: note 20 sections 2 and 8, note 16 section 5.

## F3 — Performance and timeout — PENDING (cold boot BLOCKED_PHYSICAL)

- Objective: a budget for the single 60 s clock (first sensor message -> `ACCEPTED`) with distributions, and the separate clocks next to it.
- Pending:
  1. Decision 1 of note 20 confirmed by Narcís (one clock, no restart, `LOCALIZED` and cold boot separate).
  2. Per-stage time distribution over all usable windows on the frozen candidate (med / p95 / max, N stated), search time against map size (eco01 55 k points, hospital 67 k).
  3. Noise floor: 10 identical repeats of 2 windows (S2 and S3).
  4. CPU / PSS / temperature on the NX with PSS per process; the same on the Orin Nano 8 GB if it is a deployment option.
  5. Sensitivity of the outcome to CPU contention (`--cpus 4`, `--cpus 2`): does the deadline still hold, does anything else change.
  6. Cold boot (clock A): 5 supervised power cycles, stopwatch and logs (H1.13). BLOCKED_PHYSICAL.
- Evidence: tables in `results/`, one figure of accept time per stage.
- Closing criterion: the three clocks reported separately with N and worst case; every attempt that cannot meet 60 s shown to end without a pose; margin to the deadline stated for S2 and for S3; Nano measured or explicitly excluded.
- Links: note 16 sections 6-7, note 15.

## F4 — Physical calibration — BLOCKED_PHYSICAL

- Objective: replace the provisional mount (height 0.53 m, identity extrinsic, covariance 0.15 m / 5 deg) by measured values and a written `base_footprint` definition.
- Pending: Narcís defines `base_footprint` (decision 3); tape / CAD measurements of note 17 section C; data checks D1-D4; decision on the IMU sample-count rule (decision 2) once the live stream health is known; remount (sensor looking down) and a new map.
- Evidence: filled table of note 17 section C with who measured and how; `metrics.json` `mount.status` changed from `PROVISIONAL_not_measured`.
- Closing criterion: measured values with their uncertainty in the repo, launch defaults updated, the handoff error budget recomputed, a new candidate name issued.
- Links: [17-calibracion-montaje.md](17-calibracion-montaje.md).

## F5 — Independent benchmark — BLOCKED_PHYSICAL (recording sessions)

- Objective: the first reliability statement: correct accept, wrong accept, time, on data never used for development.
- Pending: record the sessions of note 18 (at least 30 independent stationary blocks for S2, 30 for S3, negatives outside the map and against another building's map); build maps and references from a different session than the queries; freeze commit, image IDs and evaluator before opening Q-eval; one run of the full set.
- Evidence: per-cell counts with Wilson 95 % bounds, blocks and starts separately, error and time with med / p95 / max, an overlay plot for every wrong accept, invalid trials listed with cause.
- Closing criterion: note 18 section 6 executed once on the frozen candidate, results reported as they are. A change after the run consumes the set.
- Links: [18-plan-datos-independientes-y-validacion-fisica.md](18-plan-datos-independientes-y-validacion-fisica.md), protocol 02.

## F6 — H1 / H2 validation — BLOCKED_PHYSICAL

- Objective: H1 = live sensors, robot still, motors off. H2 = S3 with a person moving the chair. No autonomous motion, no Nav2 goals.
- Pending: preconditions (F4 done, motors disabled, a person at the chair); checklist H1.1-H1.14 and H2.1-H2.5 of note 18 section 7, expected outcome written before each test; in particular the live stamp base of `/initialpose` (NV in note 20), live stream loss, cold boot, supervisor reaction time against the 38 s grant limit.
- Evidence: `metrics.json`, `launch.log`, RViz capture and a tape-measured pose per test.
- Closing criterion: every H1 row executed with its outcome recorded -> `VALIDATED_ON_HARDWARE_STATIONARY`; every H2 row -> `VALIDATED_WITH_SUPERVISED_MOTION`. A failed row stays failed in the table.
- Links: note 18 section 7.

## F7 — Documentation and handover to Narcís — PENDING

- Objective: someone else can rebuild, run and judge the candidate.
- Pending: build recipe of `register_v2` and of `socialtech:reloc-online` in the repo, images exported or rebuilt from a stated commit (O8); one results document with every number pointing to its data and command; list of invalid runs and of our own mistakes; decisions taken by Narcís recorded with date; overlay figures for visual judgement; PR to `feat/nav-bench-a` (push only when the human says so).
- Closing criterion: a clean checkout plus the documented steps reproduces S2 / S3 / negative on another Jetson or after a reflash; Narcís has reviewed the figures.
- Links: AGENTS.md "The standard", notes 15-20.
