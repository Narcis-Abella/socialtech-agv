#!/usr/bin/env bash
# NX host: the gate against the REAL tracer_base with the bypass closed. No robot, no physical CAN.
#   cell  (--network none + NET_ADMIN): vcan0, tracer_base, gate_outlet.py, can_deadman.py, candump. Mounts the socket directory.
#   gate  (--network none): nav_gate.py only. Mounts the socket directory.
#   graph (network namespace of `gate`, NO socket directory): velocity source, permit stub, the rogue publishers = "every other ROS node".
# vcan0 exists only inside the cell's namespace; the script aborts if a can0 is visible there, and records the host can0 counters
# before and after. Host needs the vcan module (sudo modprobe vcan).
# usage: R=<run root> [FINAL=gate|driver] gate_cell_test.sh OUT_DIR(new)      FINAL = which process is killed last, while open
set -u
R=${R:?run root}; out=$1; img=socialtech:gate-can; T=/tools/global_reloc_3d
mkdir -p "$out/sock" "$out/ctl" || exit 1
ip -s -d link show can0 > "$out/host_can0_before.txt" 2>&1
id=$$
docker run -d --rm --name cell-$id --network none --cap-add NET_ADMIN -v $R/tools:/tools:ro -v "$out":/out -v "$out/sock":/sock $img bash $T/gate_cell_inner.sh cell > /dev/null
docker run -d --rm --name gate-$id --network none -v $R/tools:/tools:ro -v "$out":/out -v "$out/sock":/sock $img bash $T/gate_cell_inner.sh gate > /dev/null
sleep 6
docker run --rm --name graph-$id --network container:gate-$id -e FINAL=${FINAL:-gate} -v $R/tools:/tools:ro -v "$out":/out $img bash $T/gate_cell_inner.sh graph > "$out/graph.log" 2>&1
touch "$out/ctl/stop"; sleep 4
docker kill gate-$id cell-$id > /dev/null 2>&1
ip -s -d link show can0 > "$out/host_can0_after.txt" 2>&1
cmp -s "$out/host_can0_before.txt" "$out/host_can0_after.txt" && echo HOST_CAN0_UNCHANGED | tee "$out/host_can0_unchanged.txt"
python3 -B $R/tools/global_reloc_3d/gate_cell_check.py "$out" | tee "$out/result.json"
