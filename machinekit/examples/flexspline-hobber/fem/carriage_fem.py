"""X carriage (GE-X-carriage, milled steel 20 mm deck + 25 mm ribs): front-tie vertical stiffness and column-sole tilt stiffness, solid FE.

Same two numbers as `../frame_stiffness.py` (Mindlin plate, ribs as lumped thickness), on the real solid (holes under 20 mm filled so they do not
drive the mesh: volume +0.9 %, a small stiffness overestimate):
  front tie   1 kN on each of the two front block pads (x = 102, y = +-220; 83 x 48 on the z = 260 underside, spread by area), the column sole
              (x 276..376, |y| <= 190 on the z = 280 top) held in z; stiffness = 1 kN / mean pad displacement (N/um).
  sole tilt   the sole top tied to a rigid master (x 326, z 280) that may only move in z and rotate about y; all four block pads held; 1 kN m about
              y; stiffness = moment / rotation (kN m/rad). `pads_uz` holds the pads in z only (the plate model's assumption), `pads_xyz` holds them in
              all directions (blocks bolted to the rails).
"""
import json
import sys
import numpy as np
import fe

SOLE = (276.0, 376.0, -190.0, 190.0)
PADS = [(x, s * 220.0) for x in (102.0, 300.0) for s in (-1, 1)]
PAD_HALF = (41.5, 24.0)
Z_BLOCK, Z_SOLE = 260.0, 280.0


def surface_nodes(nodes, ids, z, box):
    p = nodes[ids]
    sel = (np.abs(p[:, 2] - z) < 1e-6) & (p[:, 0] >= box[0] - 1e-6) & (p[:, 0] <= box[1] + 1e-6) & (p[:, 1] >= box[2] - 1e-6) & (p[:, 1] <= box[3] + 1e-6)
    return [int(i) for i in ids[sel]]


def face_weights(conn, nodes, sel_nodes, z):
    """Area weights (6-node triangle faces on the plane z) of the nodes in sel_nodes."""
    sel = set(sel_nodes)
    seen = {}
    for e in conn:
        for f in ((0, 1, 2, 4, 5, 6), (0, 1, 3, 4, 9, 7), (1, 2, 3, 5, 8, 9), (0, 2, 3, 6, 8, 7)):
            ids = e[list(f)]
            if np.all(np.abs(nodes[ids, 2] - z) < 1e-6):
                seen[tuple(sorted(ids[:3]))] = ids
    w = {}
    for ids in seen.values():
        if not all(i in sel for i in ids):
            continue
        p = nodes[ids[:3]]
        a = 0.5 * np.linalg.norm(np.cross(p[1] - p[0], p[2] - p[0]))
        for n in ids[3:]:
            w[n] = w.get(n, 0.0) + a / 3.0
    return w


def primitive_step(path, ribbed):
    """The plate model's geometry (frame_stiffness.py `after_flat` / `after_ribbed`) as solids: 20 mm deck x 60..386, |y| <= 245, window x <= 140, |y| <= 150,
    and for `ribbed` four 20 wide, 25 deep ribs under the deck at |y| = 130 and 170 for x >= 140."""
    gmsh = fe.import_gmsh()
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("p")
        o = gmsh.model.occ
        deck = o.addBox(60, -245, 260, 326, 490, 20)
        window = o.addBox(60, -150, 259, 80, 300, 22)
        parts = [(3, deck)]
        tools = [(3, window)]
        out, _ = o.cut(parts, tools)
        if ribbed:
            ribs = [(3, o.addBox(140, yc - 10, 235, 246, 20, 25)) for yc in (-170.0, -130.0, 130.0, 170.0)]
            out, _ = o.fuse(out, ribs)
        o.synchronize()
        gmsh.write(path)
        return sum(o.getMass(3, t) for _, t in out)
    finally:
        gmsh.finalize()


def run(export_dir, size=14.0, curvature=0, method="auto", geometry="step"):
    export = json.load(open(f"{export_dir}/hobber-fem-export.json"))
    holes = [h for p in export["parts"] if p["name"] == "GE-X-carriage" for h in p["holes"]]
    import os
    tmp = os.environ.get("TMPDIR", "/tmp")
    filled = f"{tmp}/GE-X-carriage-filled.step"
    if geometry == "step":
        v1, v0 = fe.filled_step(f"{export_dir}/GE-X-carriage.step", holes, filled)
    else:
        filled = f"{tmp}/GE-X-carriage-{geometry}.step"
        v1 = v0 = primitive_step(filled, geometry == "ribbed")
    pts, (conn,) = fe.mesh_parts([filled], size, min_size=size / 2, curvature=curvature)
    ids = fe.used_nodes(conn)
    out = {"geometry": geometry, "size": size, "elements": len(conn), "dofs": 3 * len(ids), "volume_step": v0, "volume_filled": v1}
    pads = [surface_nodes(pts, ids, Z_BLOCK, (px - PAD_HALF[0], px + PAD_HALF[0], py - PAD_HALF[1], py + PAD_HALF[1])) for px, py in PADS]
    sole = surface_nodes(pts, ids, Z_SOLE, SOLE)
    out["pad_nodes"] = [len(p) for p in pads]
    out["sole_nodes"] = len(sole)

    def model():
        m = fe.Model()
        m.add_nodes(pts)
        m.add_body("carriage", conn, fe.STEEL)
        return m

    # ---- front tie
    m = model()
    for n in sole:
        m.fix(n, (2,))
    a, b = sorted(sole, key=lambda n: pts[n][0])[0], sorted(sole, key=lambda n: pts[n][0])[-1]
    m.fix(a, (0, 1))
    m.fix(b, (1,))
    front = [pads[i] for i, (px, _) in enumerate(PADS) if px == 102.0]
    load, weights = {}, []
    for pad in front:
        w = face_weights(conn, pts, pad, Z_BLOCK)
        tot = sum(w.values())
        for n, a_ in w.items():
            load[3 * n + 2] = 1000.0 * a_ / tot
        weights.append(w)
    K, M, vols = m.system()
    T, _ = m.reduction()
    u = m.solve_static(load, K, T, method=method)
    num = sum(sum(u[3 * n + 2] * a_ for n, a_ in w.items()) / sum(w.values()) for w in weights) / len(weights)
    out["front_tie_N_per_um"] = 1000.0 / (abs(num) * 1000.0)
    # compliance-based equivalent (work done by the loads)
    work = sum(f * u[d] for d, f in load.items())
    out["front_tie_work_N_per_um"] = 2 * 1000.0 ** 2 / (2 * abs(work)) / 1000.0 * 1.0
    out["volume_mesh"] = vols["carriage"]
    # ---- sole tilt
    for tag, dirs in (("pads_uz", (2,)), ("pads_xyz", (0, 1, 2))):
        m = model()
        for pad in pads:
            for n in pad:
                m.fix(n, dirs)
        if tag == "pads_uz":
            n0 = pads[0][0]
            n1 = pads[2][0]
            m.fix(n0, (0, 1))
            m.fix(n1, (1,))
        mi = m.add_master("sole", (326.0, 0.0, Z_SOLE))
        m.rigid(mi, sole)
        for k in (0, 1, 3, 5):
            m.fix_dof(m.master_dof(mi, k))
        K, M, _ = m.system()
        T, _ = m.reduction()
        moment = 1e6  # N mm
        u = m.solve_static({m.master_dof(mi, 4): moment}, K, T, method=method)
        theta = u[m.master_dof(mi, 4)]
        out[f"sole_tilt_kNm_per_rad_{tag}"] = moment / abs(theta) / 1e6
    return out


if __name__ == "__main__":
    d, out = sys.argv[1], sys.argv[2]
    sizes = [float(x) for x in sys.argv[3:]] or [14.0, 10.0]
    results = []
    for size in sizes:
        for geometry in (("step", "flat", "ribbed") if size == sizes[0] else ("step",)):
            results.append(run(d, size, geometry=geometry))
            json.dump(results, open(out, "w"), indent=1)
            print(json.dumps(results[-1]), flush=True)
