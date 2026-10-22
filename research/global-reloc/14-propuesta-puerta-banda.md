# Proposal for Narcís: band-limited support as an extra acceptance gate (not applied)

Date 2026-10-09. Status: **proposal**. Today's acceptance rule (`verify.decide`) is unchanged and remains the only rule used in every result. Evidence: note 13 (session 1 and session 2 sections). All numbers are DEVELOPMENT, measured truth-free on frozen outputs unless marked.

## Problem (V)
- Lead (≥0.10 support over a distinct runner-up) is the sole barrier in 16 of 136 negatives with a hypothesis (cross-map, 8 m hole, and their 20 %-dropout perturbations). Support alone is the sole barrier in 1.
- All-point support (≥0.70 within 0.15 m) reaches 0.70–0.82 on wrong maps or holes: floor and ceiling (horizontal surfaces) match almost anywhere at the same heights.
- Temporal confirmation agrees on wrong poses under aliasing (hospital holes: 0.02 m / 0.34° agreement 13.8 m from truth).
- In one perturbed positive (hospital w03, seed 9) the primary was a wrong pose and only the lead stopped it.
- RMSE (≤0.10 m over points within 0.15 m) sits at 0.08–0.10 for right and wrong hypotheses alike. It caused one false reject and never blocked a negative on its own.

## Proposal
1. Add, without removing anything: **band support** ≥ T_band. This is the fraction of check points whose map-frame height under the hypothesis lies in 0.10–1.80 m (the PGM obstacle band; map z = 0 is the floor by construction of `ply_to_map`) and that have map geometry within 0.15 m. This is protocol §145 in spirit. Difference: the band is cut in the map frame under the hypothesis instead of from a calibrated sensor height, which is not available (§147, NV-P). Narcís decides whether that substitution is acceptable.
2. Choose T_band **on data not used so far** (a new bag/session, or held-out windows), never from the numbers below.
3. Optional and separate: review the RMSE gate (low information, see above).

## Observed separation (for orientation only: these are the scored data)
| Set | N | Band support |
|---|---:|---|
| Accepted correct positives, session 1 (same-run) | 14 | 0.97–1.00 |
| Accepted correct positives, session 2 (incl. 10 cross-session) | 27 | ≥0.962 |
| Cross-map primaries, session 1 | 15 | ≤0.65 |
| Single-gate negatives (+ eco03 c03 cross) | 18 | ≤0.715 |

Expected effect (I): a second, geometrically different barrier in the 17 single-gate cases. Risk: more false rejects where the band is sparse (long-range, ceiling-dominated views of the upward-facing Mid-360). Re-check after the downward remount.

## Decision needed
Accept / reject / modify; calibration data source; whether the map-frame band replaces the sensor-height band until the mount is calibrated.
