# gn_loc shutdown race

SIGINT while the bond is alive used to abort `gn_loc_node` (`rclcpp::exceptions::RCLError: Couldn't initialize rcl timer handle: the given context is not valid`, exit -6): the bond thread created a timer after the context turned invalid. Fixed by the pre-shutdown hook in `gn_loc_node.cpp` (as `nav2_util::LifecycleNode`): deactivate, cleanup, then `bond_.reset()` whatever the state (an ACTIVE -> FINALIZED `shutdown` keeps the bond otherwise).

The race needs a squeezed CPU (2 cores, 3 busy loops). Every stop must reach TRACKING before the SIGINT (the pose is republished until then), else it is INVALID. Hang = launch escalation (SIGTERM/SIGKILL) or harness SIGKILL. gn_loc `exit -2` (a second forwarded SIGINT after `rclcpp::shutdown()`) is counted apart: it happens in base too and is not gn_loc's.

Measured on the AGX, 30 stops per cell (abort -6 / hang -9):

| cell | base | fixed |
|---|---|---|
| SIGINT, 10 Hz scans | 22 / 0 | 0 / 0 |
| SIGINT, 50 Hz scans | 7 / 0 | 0 / 0 |
| `lifecycle shutdown` from ACTIVE, then SIGINT | 29 / 0 | 0 / 0 |
| 10 Hz, scan callback delayed 40 ms (instrumented) | n/a | listener reset under `mtx_`: 4 hangs; before the lock: 0 |

Manual test (docker, an image with Nav2; not part of `colcon test`). Build `gn_loc` into `<this dir>/install_<name>`, then `CELLS="fix10:fix:10:sigint:0 lc_fix:fix:10:lcshutdown:0" N=30 ./variants.sh` (cell = `name:install:scan_hz:mode:delay_ms`; results in `out_<name>`). `GN_SCAN_DELAY_MS` needs an instrumented build.
