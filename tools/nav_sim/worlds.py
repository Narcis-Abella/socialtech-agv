"""Synthetic test worlds: one spec (boxes with pose and size) gives the Gazebo world, the occupancy map (rasterized from the same boxes, so it is registered to the world by construction) and the
start/goal trials with their reference shortest length. Boxes marked mapped=False are in the world but not in the map (unmapped obstacles). The world is called "loyola" so the simulator launch
(bridges, spawn) works unchanged: ros2 launch hospital-agv-sim simulation.launch.py world:=<abs path>/world.sdf.
usage: worlds.py <out dir>   writes one directory per world (world.sdf, map.pgm, map.yaml, scenario.json)"""
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

import gt_map
import scenarios as sc

HEADER = '<?xml version="1.0" ?>\n<sdf version="1.9">\n  <world name="loyola">\n\n    <physics name="1ms" type="ignored">\n      <max_step_size>0.001</max_step_size>\n      <real_time_factor>1.0</real_time_factor>\n    </physics>\n\n    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">\n    </plugin>\n\n    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n      <render_engine>\n        ogre2\n      </render_engine>\n    </plugin>\n\n    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">\n    </plugin>\n\n    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">\n    </plugin>\n\n    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">\n    </plugin>\n\n    <scene>\n      <ambient>0.3 0.3 0.3 1.0</ambient> \n      <background>0.8 0.8 0.8 1.0</background>\n      <grid>false</grid>\n      <origin_visual>false</origin_visual>\n      <shadows>false</shadows> \n    </scene>\n\n    <light type="directional" name="sun">\n      <cast_shadows>false</cast_shadows>\n      <pose>0 0 10 0 0 0</pose>\n      <diffuse>0.8 0.8 0.8 1</diffuse>\n      <specular>0.2 0.2 0.2 1</specular>\n      <attenuation>\n        <range>1000</range>\n        <constant>0.9</constant>\n        <linear>0.01</linear>\n        <quadratic>0.001</quadratic>\n      </attenuation>\n      <direction>-0.25 0.25 -0.9</direction>\n    </light>'

BOX = """
    <model name="b_{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>"""

GROUND = """
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><ambient>0.2 0.3 0.4 1</ambient><diffuse>0.2 0.3 0.4 1</diffuse></material></visual>
      </link>
    </model>"""


def rasterize(spec):
    """(occupied grid, origin) of the mapped boxes; row 0 = top (max y), cell size spec['res']."""
    x0, y0, x1, y1 = spec["bounds"]
    res = spec["res"]
    h, w = int(round((y1 - y0) / res)), int(round((x1 - x0) / res))
    xs = x0 + (np.arange(w) + 0.5) * res
    ys = y0 + (h - 1 - np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros((h, w), bool)
    for b in spec["boxes"]:
        if not b.get("mapped", True):
            continue
        dx, dy = X - b["x"], Y - b["y"]
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        occ |= (np.abs(c * dx + s * dy) <= b["sx"] / 2) & (np.abs(-s * dx + c * dy) <= b["sy"] / 2)
    return occ, (float(x0), float(y0))


def free_unknown(occ, origin, res, seed):
    """Free = the 4-connected free region that contains `seed`; everything else that is not occupied is unknown."""
    r, c = sc.to_cell(seed[0], seed[1], origin, res, occ.shape[0])
    lab, _ = ndimage.label(~occ)
    if occ[r, c]:
        raise ValueError("the seed is inside an obstacle")
    return {"occupied": occ, "free": lab == lab[r, c]}


def sdf(spec):
    boxes = "".join(BOX.format(name=b["name"], x=b["x"], y=b["y"], z=b["sz"] / 2, yaw=b["yaw"], sx=b["sx"], sy=b["sy"], sz=b["sz"]) for b in spec["boxes"])
    return HEADER + "\n" + GROUND + boxes + "\n  </world>\n</sdf>\n"


def scenario(spec, clearance=0.29):
    """The spec's trials with the reference shortest length through the map eroded by `clearance` (half the footprint width); reachable=False when there is no such path."""
    occ, origin = rasterize(spec)
    est = free_unknown(occ, origin, spec["res"], spec["seed"])
    h = occ.shape[0]
    free = sc.eroded_free(~est["free"], clearance, spec["res"])
    out = []
    for t in spec["trials"]:
        a, b = sc.to_cell(t["start"][0], t["start"][1], origin, spec["res"], h), sc.to_cell(t["goal"][0], t["goal"][1], origin, spec["res"], h)
        length = sc.shortest_length(free, a, b, spec["res"])
        out.append({**t, "shortest": round(length, 3) if math.isfinite(length) else None, "reachable": math.isfinite(length)})
    return out


def build(spec, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    occ, origin = rasterize(spec)
    gt_map.write_map(free_unknown(occ, origin, spec["res"], spec["seed"]), os.path.join(out_dir, "map"), spec["res"], origin)
    open(os.path.join(out_dir, "world.sdf"), "w").write(sdf(spec))
    trials = scenario(spec)
    json.dump({"name": spec["name"], "map": "map.yaml", "world": "world.sdf", "trials": trials}, open(os.path.join(out_dir, "scenario.json"), "w"), indent=1)
    return trials


def box(name, x, y, sx, sy, yaw=0.0, sz=2.5, mapped=True):
    return {"name": name, "x": x, "y": y, "yaw": yaw, "sx": sx, "sy": sy, "sz": sz, "mapped": mapped}


def doors_spec(widths=(0.8, 0.9, 1.0, 1.2)):
    """A 20 x 10 m room split by a wall at x = 10 with four doors; one trial goes straight through each door."""
    ys = [-3.5, -1.2, 1.2, 3.5]
    boxes = [box("n", 10.0, 5.05, 20.2, 0.1), box("s", 10.0, -5.05, 20.2, 0.1), box("w", -0.05, 0.0, 0.1, 10.2), box("e", 20.05, 0.0, 0.1, 10.2)]
    edges, lo = [], -5.0
    for y, wd in zip(ys, widths):
        if y - wd / 2 > lo:
            edges.append((lo, y - wd / 2))
        lo = y + wd / 2
    edges.append((lo, 5.0))
    boxes += [box(f"d{i}", 10.0, (a + b) / 2, 0.2, b - a) for i, (a, b) in enumerate(edges)]
    return {"name": "doors", "res": 0.05, "bounds": [-1.0, -6.0, 21.0, 6.0], "seed": [2.0, 0.0], "boxes": boxes,
            "doors": [{"y": y, "width": wd} for y, wd in zip(ys, widths)],
            "trials": [{"id": f"d{int(round(wd * 100)):03d}", "start": [2.0, y, 0.0], "goal": [18.0, y, 0.0]} for y, wd in zip(ys, widths)]}


def corridor_spec(gap, name):
    """A 24 m x 2 m corridor with an UNMAPPED obstacle (0.4 m deep) at x = 12 leaving a gap of `gap` metres along the north wall (gap 0 closes it); trial end to end."""
    sy = 2.0 - gap
    boxes = [box("n", 12.0, 1.05, 24.2, 0.1), box("s", 12.0, -1.05, 24.2, 0.1), box("w", -0.05, 0.0, 0.1, 2.2), box("e", 24.05, 0.0, 0.1, 2.2)]
    if sy > 0:
        boxes.append(box("obstacle", 12.0, -1.0 + sy / 2, 0.4, sy, sz=1.0, mapped=False))
    return {"name": name, "res": 0.05, "bounds": [-1.0, -3.0, 25.0, 3.0], "seed": [2.0, 0.0], "boxes": boxes,
            "trials": [{"id": "end_to_end", "start": [2.0, 0.0, 0.0], "goal": [22.0, 0.0, 0.0]}, {"id": "back", "start": [22.0, 0.0, math.pi], "goal": [2.0, 0.0, math.pi]}]}


def turn_spec():
    """An L-shaped corridor 1.6 m wide: 12 m east then 12 m north."""
    boxes = [box("s", 6.0, -0.05, 12.2, 0.1), box("w", -0.05, 0.85, 0.1, 1.9), box("n1", 5.15, 1.65, 10.3, 0.1), box("n2", 10.35, 6.85, 0.1, 10.5), box("e", 12.05, 6.0, 0.1, 12.2),
             box("top", 11.2, 12.05, 1.8, 0.1)]
    return {"name": "turn_l", "res": 0.05, "bounds": [-1.0, -1.0, 13.0, 13.0], "seed": [1.0, 0.8], "boxes": boxes,
            "trials": [{"id": "l_out", "start": [1.0, 0.8, 0.0], "goal": [11.2, 11.0, math.pi / 2]}, {"id": "l_back", "start": [11.2, 11.0, -math.pi / 2], "goal": [1.0, 0.8, math.pi]}]}


def rows_spec():
    """A 36 x 16 m hall with 3 rows x 5 columns of identical 2.0 x 0.8 m blocks (1.5 m high) every 6 m along x: the perceptual aliasing of the real eco_-1 bags (rows of near-identical objects). Beyond the
    12 m of the simulated lidar the hall ends are out of reach, so in the middle only the parallel walls and the periodic blocks are visible. Trials run end to end along the four lanes, both ways."""
    boxes = [box("n", 18.0, 8.05, 36.2, 0.1), box("s", 18.0, -8.05, 36.2, 0.1), box("w", -0.05, 0.0, 0.1, 16.2), box("e", 36.05, 0.0, 0.1, 16.2)]
    boxes += [box(f"blk{i}{j}", 6.0 * (i + 1), 4.0 * (j - 1), 2.0, 0.8, sz=1.5) for i in range(5) for j in range(3)]
    lanes = [-6.0, -2.0, 2.0, 6.0]
    trials = [{"id": f"fwd{k}", "start": [2.0, y, 0.0], "goal": [34.0, y, 0.0]} for k, y in enumerate(lanes)]
    trials += [{"id": f"back{k}", "start": [34.0, y, math.pi], "goal": [2.0, y, math.pi]} for k, y in enumerate(lanes)]
    return {"name": "rows", "res": 0.05, "bounds": [-1.0, -9.0, 37.0, 9.0], "seed": [2.0, -6.0], "boxes": boxes, "trials": trials}


ALL = {"doors": doors_spec, "corridor_gap12": lambda: corridor_spec(1.2, "corridor_gap12"), "corridor_gap09": lambda: corridor_spec(0.9, "corridor_gap09"),
       "corridor_blocked": lambda: corridor_spec(0.0, "corridor_blocked"), "turn_l": turn_spec, "rows": rows_spec}


if __name__ == "__main__":
    for name, make in ALL.items():
        trials = build(make(), os.path.join(sys.argv[1], name))
        print(name, [(t["id"], t["shortest"]) for t in trials])
