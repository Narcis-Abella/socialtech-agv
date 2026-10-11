| scenario | variant | N | correct_accept | wrong_accept | false_reject | correct_reject | error_no_pose | unscorable | wrong_accept 95 % upper (Wilson) |
|---|---|---|---|---|---|---|---|---|---|
| s3 | cross | 7 | 0 | 0 | 0 | 6 | 1 | 0 | 0.35 |
| s3 | hole | 3 | 0 | 0 | 0 | 3 | 0 | 0 | 0.56 |
| s3 | own | 22 | 18 | 0 | 0 | 0 | 1 | 3 | 0.17 |

Latency of correct accepts, S3 positives (median / p95 / max seconds), N=18:

| quantity | median / p95 / max |
|---|---|
| imu_time | 2.96 / 4.86 / 7.40 (N=18) |
| lio_time | 0.58 / 0.68 / 0.68 (N=18) |
| submap_time | 18.78 / 27.25 / 27.42 (N=18) |
| submap_build | 3.50 / 5.61 / 5.65 (N=18) |
| fft_time | 7.14 / 9.56 / 9.73 (N=18) |
| gicp_time | 3.84 / 4.64 / 4.64 (N=18) |
| verification_time | 0.93 / 1.03 / 1.05 (N=18) |
| search_wall | 11.74 / 15.17 / 15.34 (N=18) |
| handoff_time | 5.30 / 8.58 / 9.81 (N=18) |
| total_time_protocol | 39.84 / 47.14 / 47.77 (N=18) |
| localized_total | 45.81 / 52.58 / 53.43 (N=18) |
| cpu_mean_pct | 68.60 / 80.30 / 80.30 (N=18) |
| cpu_p95_pct | 212.60 / 233.62 / 235.40 (N=18) |
| rss_peak_mb | 495.45 / 588.64 / 594.00 (N=18) |
| pss_peak_mb | 339.10 / 415.64 / 417.60 (N=18) |
| search_child_maxrss_mb | 87.25 / 93.00 / 93.00 (N=18) |
| drops_imu_pct | 7.14 / 8.24 / 8.24 (N=18) |
| drops_witness_imu_pct | 7.03 / 8.23 / 8.24 (N=18) |
| xy_err_m | 0.03 / 0.08 / 0.08 (N=18) |
| yaw_err_deg | 0.16 / 1.21 / 1.21 (N=18) |
| amcl_xy_median_m | 0.03 / 0.11 / 0.11 (N=18) |
