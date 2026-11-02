# gn_loc shutdown race

SIGINT while the bond is alive used to abort `gn_loc_node` (`rclcpp::exceptions::RCLError: Couldn't initialize rcl timer handle: the given context is not valid`, exit -6): the bond thread created a timer after the context turned invalid. Fixed by the pre-shutdown hook in `gn_loc_node.cpp` (as `nav2_util::LifecycleNode`).

The race needs a squeezed CPU. Measured on the AGX (20 stops each, 2 cores, 3 busy loops):

| variant | before | after |
|---|---|---|
| A: bond only (`SYNTH=0 BOND=1`) | 18 / 20 | 0 / 20 |
| C: bond + synthetic scans and TF (`SYNTH=1 BOND=1`) | 12 / 20 | 0 / 20 |
| B: scans and TF, no bond (`SYNTH=1 BOND=0`) | 0 / 20 | 0 / 20 |

Manual test (needs docker and an image with Nav2; it is not part of `colcon test`). Build `gn_loc` into `<this dir>/install_x`, then `VARIANTS="A C" ./variants.sh install_x` (or one variant: `docker run --rm --cpuset-cpus 0-1 -e HOG=3 -e SYNTH=0 -e BOND=1 -v "$PWD:/g" IMAGE bash /g/harness.sh install_x 20 out_A`). A stop is clean only if the hook ran and nothing aborted, segfaulted or needed a SIGKILL. A real SIGINT matters: a background job of a non-interactive shell ignores it, hence `timeout -s INT` in the foreground.
