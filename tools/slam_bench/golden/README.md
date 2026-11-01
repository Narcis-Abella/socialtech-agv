# Golden files of the make_map refactor

`r03.sha256` pins the output of the pipeline as it was before the refactor (`ply_to_map.py` then `map_visibility.py --engine cpp --res 0.025 --ceil 1.80`, the arguments of `make_map.sh`) on frozen inputs: `world.ply` and `alidarState.txt` of `mapa_20261030_01_opt`, bag `mapeo_20261030_2110` (366 s, CustomMsg).
The new pipeline, with the ghost test limited to the old band (`ghost_band=(0.10, 1.80)`), must reproduce `map.pgm` and `map.yaml` byte for byte, and its `map_clean.ply` filtered to the obstacle band over occupied cells must equal the old `map_clean.ply`.

The deployed `~/maps/mapa_20261030_01_r03/map.pgm` on the Nano is NOT this golden (sha256 `5fdaa02c...`, 48 497 occupied cells vs 39 179): it was made with other options or an older version of the tools. The golden is what the code of `main` produces, which is what a refactor must preserve.

Reproduce the old output on the AGX (frozen inputs in `~/exp_fullheight/in`, tools of the commit above mounted on `/tools`):

    docker run --rm -v TOOLS:/tools:ro -v BAG:/bag:ro -v IN:/in:ro -v OUT:/out socialtech:mapper bash -c '
      source /ros_env.sh; cd /out
      python3 -B /tools/ply_to_map.py /in/world.ply base --ceil 1.80 --res 0.025 --traj /in/alidarState.txt
      python3 -B /tools/map_visibility.py /bag /in/alidarState.txt /in/world.ply map --topic /livox/lidar --ceil 1.80 --res 0.025 --ply-out map_clean.ply --engine cpp'

The old tools are deleted at the end of the refactor; this recipe needs the commit `a327b50`.

## Check the new pipeline against it

    docker run --rm -v TOOLS:/t:ro -v BAG:/bag:ro -v IN:/in:ro -v OUT:/out socialtech:mapper bash -c '
      source /ros_env.sh; python3 -B /t/slam_bench/golden/check.py /bag /in/world.ply /in/alidarState.txt /out'

It runs `make_map.build` with the ghost test limited to the old band (`ghost_band=(0.10, 1.80)`), rebuilds the old `map_clean.ply` from the new whole-height one and prints `GOLDEN OK` when `map.pgm`, `map.yaml` and that cloud have the sha256 of `r03.sha256`. First run 2026-11-01 on the AGX (`make-map-pipeline` branch, `f5b4b56`): all three equal.
