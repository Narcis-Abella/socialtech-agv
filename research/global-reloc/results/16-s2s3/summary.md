| config | start | variant | N | classes | accepted in S2 / S3 | accept at s (med/p95/max) | LOCALIZED at s | attempt end at s | xy err m (max) | yaw err deg (max) |
|---|---|---|---|---|---|---|---|---|---|---|
| s2s3x1 | motion_follows | own | 17 | correct_accept 15, false_reject_timeout 1, unscorable_reference_gap 1 | 3 / 13 | 39.1 / 46.7 / 47.6 (N=16) | 44.7 / 52.2 / 53.4 (N=16) | 43.5 / 52.1 / 53.4 (N=17) | 0.084 | 1.195 |
| s2s3x1 | stationary_episode | own | 5 | correct_accept 5 | 0 / 5 | 45.2 / 47.0 / 47.4 (N=5) | 50.4 / 55.9 / 56.7 (N=5) | 50.4 / 55.9 / 56.7 (N=5) | 0.075 | 0.272 |
| s2s3x1na | stationary_episode | own | 2 | false_reject_timeout 2 | 0 / 0 | NA (N=0) | NA (N=0) | 38.2 / 38.2 / 38.2 (N=2) | NA | NA |
| s2s3x3 | motion_follows | cross | 8 | correct_reject 8 | 0 / 0 | NA (N=0) | NA (N=0) | 42.4 / 45.1 / 45.1 (N=8) | NA | NA |
| s2s3x3 | motion_follows | hole | 3 | correct_reject 3 | 0 / 0 | NA (N=0) | NA (N=0) | 44.2 / 44.8 / 44.8 (N=3) | NA | NA |
| s2s3x3 | motion_follows | own | 17 | correct_accept 15, error_no_pose 1, false_reject_timeout 1 | 14 / 1 | 17.7 / 25.7 / 34.0 (N=15) | 23.0 / 30.8 / 39.2 (N=15) | 23.1 / 39.5 / 40.5 (N=17) | 0.068 | 0.508 |
| s2s3x3 | stationary_episode | cross | 5 | correct_reject 5 | 0 / 0 | NA (N=0) | NA (N=0) | 46.1 / 47.3 / 47.6 (N=5) | NA | NA |
| s2s3x3 | stationary_episode | own | 10 | correct_accept 10 | 8 / 2 | 16.6 / 41.9 / 42.0 (N=10) | 21.8 / 47.3 / 47.4 (N=10) | 21.8 / 47.3 / 47.4 (N=10) | 0.067 | 0.417 |
| s2s3x5 | stationary_episode | own | 5 | correct_accept 5 | 4 / 1 | 18.4 / 37.7 / 41.8 (N=5) | 24.2 / 43.0 / 47.1 (N=5) | 24.2 / 43.0 / 47.1 (N=5) | 0.067 | 0.272 |
| s3 | motion_follows | cross | 1 | correct_reject 1 | 0 / 0 | NA (N=0) | NA (N=0) | 45.0 / 45.0 / 45.0 (N=1) | NA | NA |
| s3 | motion_follows | own | 5 | correct_accept 5 | 0 / 0 | 38.2 / 39.9 / 39.9 (N=5) | 43.4 / 45.8 / 45.8 (N=5) | 43.4 / 45.8 / 45.8 (N=5) | 0.074 | 1.206 |

imu_gate_s: 2.6 / 8.3 / 39.7 (N=78)
lio_ready_s: 0.6 / 0.7 / 0.7 (N=77)
s2_exposure_s: 2.9 / 5.0 / 5.0 (N=71)
s2_build_s: 0.6 / 0.9 / 0.9 (N=69)
s2_search_s: 9.4 / 12.2 / 13.4 (N=69)
s2_decided_at_s: 16.0 / 21.5 / 22.1 (N=69)
s3_acq_end_at_s: 26.5 / 30.0 / 30.0 (N=38)
s3_build_s: 4.0 / 5.5 / 5.6 (N=38)
s3_search_s: 11.3 / 15.2 / 16.0 (N=38)
handoff_confirm_s: 5.3 / 6.4 / 11.0 (N=56)
cpu_mean_pct: 81.3 / 103.9 / 107.5 (N=78)
cpu_p95_pct: 222.6 / 237.6 / 241.7 (N=78)
cpu_max_pct: 233.0 / 249.2 / 256.7 (N=78)
pss_peak_mb: 319.1 / 396.2 / 429.7 (N=78)
rss_peak_mb: 464.9 / 566.9 / 601.2 (N=78)
worker_maxrss_mb: 85.9 / 92.9 / 93.2 (N=78)
gpu_max_pct: 0.0 / 0.0 / 0.0 (N=78)
cpu_temp_max_c: 63.8 / 64.7 / 64.9 (N=78)
imu_lost_pct: 6.9 / 8.3 / 8.6 (N=78)
witness_imu_lost_pct: 6.9 / 8.3 / 8.4 (N=78)
imu_rate_hz: 200.0 / 200.0 / 200.0 (N=78)
pose published without acceptance: 0
