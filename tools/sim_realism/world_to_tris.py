"""Flatten the visual geometry of a Gazebo world (SDF) into a binary triangle soup for the Livox ray caster.
usage: world_to_tris.py WORLD.sdf MODELS_DIR OUT.tris [--skip NAME ...]

Visuals, not collisions: the gpu_lidar sees the rendered scene (collisions of the furniture are 1 cm spheres). Handles box, plane (finite size), OBJ/STL-free mesh via OBJ only,
pose chains model > link > visual and mesh <scale>. .tris layout (little endian): magic 'TRIS', u32 n_tri, then n_tri * 9 float32 (three vertices xyz, world frame).
"""
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def pose(el):
    p = el.find("pose") if el is not None else None
    v = [float(x) for x in p.text.split()] if p is not None and p.text and p.text.strip() else [0.0] * 6
    T = np.eye(4); T[:3, :3] = rot(*v[3:6]); T[:3, 3] = v[:3]
    return T


def box(sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2; c = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * h
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return np.array([[c[i] for i in t] for t in f])


def obj(path, scale):
    v, tri = [], []
    for ln in open(path, errors="ignore"):
        w = ln.split()
        if not w: continue
        if w[0] == "v": v.append([float(x) for x in w[1:4]])
        elif w[0] == "f":
            ix = [int(t.split("/")[0]) for t in w[1:]]; ix = [i - 1 if i > 0 else len(v) + i for i in ix]
            tri += [(ix[0], ix[k], ix[k + 1]) for k in range(1, len(ix) - 1)]
    v = np.array(v) * scale
    return v[np.array(tri)]


def main():
    world, models, out = sys.argv[1], Path(sys.argv[2]), sys.argv[3]; skip = sys.argv[sys.argv.index("--skip") + 1:] if "--skip" in sys.argv else []
    w = ET.parse(world).getroot().find("world"); tris = []; n_mesh = 0
    for m in w.findall("model"):
        if m.get("name") in skip: continue
        Tm = pose(m)
        for l in m.findall("link"):
            Tl = Tm @ pose(l)
            for v in l.findall("visual"):
                g = v.find("geometry"); T = Tl @ pose(v); t = None
                if g.find("box") is not None: t = box(*[float(x) for x in g.find("box/size").text.split()])
                elif g.find("plane") is not None:
                    sx, sy = [float(x) for x in g.find("plane/size").text.split()]; t = box(sx, sy, 1e-3) + np.array([0, 0, -5e-4])   # thin slab, normal +z as in the world
                elif g.find("mesh") is not None:
                    me = g.find("mesh"); uri = me.findtext("uri"); sc = [float(x) for x in me.findtext("scale", "1 1 1").split()]
                    assert uri.startswith("model://") and uri.endswith(".obj"), uri
                    t = obj(models / uri[len("model://"):], np.array(sc)); n_mesh += 1
                if t is None: continue
                tris.append((t.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]).reshape(-1, 3, 3))
    a = np.concatenate(tris).astype("<f4")
    with open(out, "wb") as f: f.write(b"TRIS"); f.write(struct.pack("<I", len(a))); f.write(a.tobytes())
    lo, hi = a.reshape(-1, 3).min(0), a.reshape(-1, 3).max(0)
    print(f"{len(a)} triangles ({n_mesh} meshes), bounds {np.round(lo, 2)} .. {np.round(hi, 2)} -> {out}")


if __name__ == "__main__":
    main()
