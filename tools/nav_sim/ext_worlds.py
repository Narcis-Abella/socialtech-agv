"""Imports a box-only SDF world (e.g. mertgulerx/world-collection, Apache-2.0: worlds/<ID>/<ID>.sdf + world.yaml) into the nav_sim world format of worlds.py: the boxes become
the structure of our own world (loyola name, our systems and physics step) and the evaluation map is rasterized from the same boxes, so it is registered to the world by construction.
Used with world-collection commit 435a4b59491c412b0872ddca5675c6d9817dd986 (git clone https://github.com/mertgulerx/world-collection into a fresh directory; nothing of it is executed); the generated worlds go to worlds_ext/ (git-ignored, regenerate).
usage: ext_worlds.py <world dir> <out dir> [name]      writes world.sdf, map.pgm/.yaml and scenario.json (the spawns of world.yaml chained S1 -> S2 -> ... as trials)
Only collision boxes of models other than the ground plane are read; a box that does not stand on the floor (z = height / 2) or has roll/pitch is refused (the 2D map would be wrong)."""
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

import worlds


def _pose(el):
    p = el.find("pose") if el is not None else None
    v = [float(x) for x in p.text.split()] if p is not None and p.text and p.text.strip() else [0.0] * 6
    return v + [0.0] * (6 - len(v))


def _planar(parent, child):
    """Pose of `child` (x y z r p y) given in the frame of `parent`, both roll/pitch free of tilt."""
    for p in (parent, child):
        if abs(p[3]) > 1e-6 or abs(p[4]) > 1e-6:
            raise ValueError("a pose with roll/pitch (tilt) is not supported")
    c, s = math.cos(parent[5]), math.sin(parent[5])
    return [parent[0] + c * child[0] - s * child[1], parent[1] + s * child[0] + c * child[1], parent[2] + child[2], 0.0, 0.0, parent[5] + child[5]]


def parse_boxes(sdf_path):
    world = ET.parse(sdf_path).getroot().find("world")
    out = []
    for model in world.findall("model"):
        if "ground" in model.get("name", "").lower():
            continue
        mp = _pose(model)
        for link in model.findall("link"):
            lp = _planar(mp, _pose(link))
            for col in link.findall("collision"):
                box = col.find("geometry/box/size")
                if box is None:
                    continue
                sx, sy, sz = (float(v) for v in box.text.split())
                x, y, z, _, _, yaw = _planar(lp, _pose(col))
                if abs(z - sz / 2) > 0.02:
                    raise ValueError(f"{col.get('name')}: the box does not stand on the floor (z {z}, height {sz})")
                out.append(worlds.box(col.get("name", f"b{len(out)}"), x, y, sx, sy, yaw=yaw, sz=sz))
    if not out:
        raise ValueError("no collision box found")
    return out


def spec_from_boxes(name, boxes, spawns, res=0.05, margin=1.0):
    xs, ys = [], []
    for b in boxes:
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            xs.append(b["x"] + c * dx * b["sx"] / 2 - s * dy * b["sy"] / 2)
            ys.append(b["y"] + s * dx * b["sx"] / 2 + c * dy * b["sy"] / 2)
    trials = [{"id": f"{a['id']}_{b['id']}", "start": [a["x"], a["y"], a["yaw"]], "goal": [b["x"], b["y"], b["yaw"]]} for a, b in zip(spawns, spawns[1:])]
    return {"name": name, "res": res, "bounds": [min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin], "seed": [spawns[0]["x"], spawns[0]["y"]], "boxes": boxes, "trials": trials}


def main(world_dir, out_dir, name=None):
    wid = os.path.basename(world_dir.rstrip("/"))
    meta = yaml.safe_load(open(os.path.join(world_dir, "world.yaml")))
    boxes = parse_boxes(os.path.join(world_dir, wid + ".sdf"))
    spec = spec_from_boxes(name or wid, boxes, meta["spawns"])
    trials = worlds.build(spec, out_dir)
    print(wid, len(boxes), "boxes;", [(t["id"], t["shortest"]) for t in trials])


if __name__ == "__main__":
    main(*sys.argv[1:4])
