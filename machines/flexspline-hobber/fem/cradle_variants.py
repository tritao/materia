"""Prototype cradle variants built from boxes (gmsh OCC) for the design study of 2026-11-06, run through the cradle FE (`cradle_fem.Cradle`).

The exported cradle (HobberFemExport) is the reference; this module builds the same cradle from boxes so that a variant (wall thickness, rib, window, floor
extension, a rest block under the servo body, extra floor bolts) can be meshed and solved in about a minute without recompiling the model. The variants of
`results/cradle-variants.json` came from here; they fit the hand model's constants (`cradle_handmodel.py`). Geometry constants are the authored frame's (mm):
seat top z 58, floor 12 / 8 thick (step at y 195), foot 18 under the wall (y 222..), wall face y 239, motor axis x -8, z 124.74.

Usage: python3 cradle_variants.py <export dir> <name> '<json params>' [joints] with params as `BASE` below plus
  fy0       floor start y (default 150)       extra  [[x0, x1, y0, y1, z0, z1], ...] boxes added to the cradle (a rest block: z1 = its top)
  rest      {"box": [x0, x1, y0, y1], "z": top, "dirs": [2], "k": N/mm or null}   the servo group is coupled to the rest block's top nodes (z only)
  bolts     [[x, y], ...] floor bolt rows for the joint model
"""
import json
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cradle_fem
import fe

C = 124.7437
BASE = dict(wall=8.0, rib=True, window=True, ribt=6.0, x0=-56.0, x1=31.0, foot=18.0, recess=3.0, ribx=None, fy0=150.0, extra=[])


def build(path, **kw):
    p = dict(BASE)
    p.update(kw)
    gmsh = fe.import_gmsh()
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("c")
        o = gmsh.model.occ
        low, thin, floor, foot, face = 58.0, 66.0, 70.0, 58.0 + p["foot"], 239.0
        rear, top = face + p["wall"], C + 47

        def box(x0, x1, y0, y1, z0, z1):
            return (3, o.addBox(x0, y0, z0, x1 - x0, y1 - y0, z1 - z0))
        rx0 = p["x0"] - 5 if p["ribx"] is None else p["ribx"]
        parts = [box(-44, 38, p["fy0"], 195, low, thin), box(p["x0"], 38, 195, 222, low, floor), box(p["x0"], 38, 222, rear, low, foot), box(p["x0"], p["x1"], face, rear, foot, top)]
        if p["rib"]:
            parts.append(box(rx0, rx0 + p["ribt"], 192, rear, low, top))
        parts.append(box(-21, 5, 224, 239.5, foot, foot + 4))  # the jack lug
        parts += [box(*e) for e in p["extra"]]
        res = o.fuse([parts[0]], parts[1:])[0]
        cuts = [box(-2, 2, p["fy0"] - 1, 236, low - 1, 90)]  # the slit through the floor and the rest block
        if p["rib"] and p["window"]:
            cuts.append(box(rx0 - 1, rx0 + p["ribt"] + 1, 209, 232, 95, 160))

        def slot(x, zc, d, depth, y0=face):
            return [(3, o.addCylinder(x, y0, zc - 3, 0, depth, 0, d / 2)), (3, o.addCylinder(x, y0, zc + 3, 0, depth, 0, d / 2)), (3, o.addBox(x - d / 2, y0, zc - 3, d, depth, 6))]
        if p["recess"] > 0:
            cuts += slot(-8, C, 70.2, p["recess"])
        cuts += slot(-8, C, 27, p["wall"] + 0.5)
        for hx, hz in [(23.8198, 156.5635), (-39.8198, 156.5635), (-39.8198, 92.9239), (23.8198, 92.9239)]:
            cuts += slot(hx, hz, 7.0, p["wall"] + 0.5)
        o.cut(res, cuts)
        o.synchronize()
        gmsh.write(path)
    finally:
        gmsh.finalize()


def run(export_dir, work, name, params, joints=("pre_hi",), flange="bolts", support="plate", size=8.0):
    params = dict(params)
    rest, bolts = params.pop("rest", None), params.pop("bolts", None)
    d = os.path.join(work, name)
    os.makedirs(d, exist_ok=True)
    shutil.copy(os.path.join(export_dir, "hobber-fem-export.json"), d)
    build(os.path.join(d, "GE-motor-cradle.step"), **params)
    export = json.load(open(os.path.join(d, "hobber-fem-export.json")))
    export.pop("cradle", None)  # the variants carry their own bolts / rest
    json.dump(export, open(os.path.join(d, "hobber-fem-export.json"), "w"))
    export_part = next(p for p in export["parts"] if p["name"] == "GE-motor-cradle")
    export_part["holes"] = []  # box variants have no floor holes to fill
    t = time.time()
    if bolts:
        cradle_fem.BOLTS = [tuple(b) for b in bolts]
    c = cradle_fem.Cradle(export, d, size, 16.0, support, flange, 4)
    if rest:
        c.pad = dict(rest)
    out = {"name": name, "params": params, "rest": rest, "bolts": bolts, "elements": [len(c.cc), len(c.sc)]}
    for j in joints:
        r = c.modes(j, 4)
        out[j] = [round(x) for x in r["frequencies_Hz"]]
    out["secs"] = round(time.time() - t)
    return out


if __name__ == "__main__":
    export_dir, name = sys.argv[1], sys.argv[2]
    params = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    joints = tuple(sys.argv[4].split(",")) if len(sys.argv) > 4 else ("pre_hi",)
    print(json.dumps(run(export_dir, os.environ.get("TMPDIR", "/tmp") + "/hobber-fem-variants", name, params, joints)), flush=True)
