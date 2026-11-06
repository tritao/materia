"""B motor cradle on the spindle seat block: first modes of the servo group on the cradle (FRAME_PLAN, `HobberFrameChecks.spindleDrive`).

Parts: GE-motor-cradle + GE-spindle-seat from the exporter (authored frame, mm). The servo group is a rigid point mass (mass, inertia of a
90 x 90 x 115 box for the motor body about the group's centre) coupled to the wall's front face (y = 239) at the four M6 flange bolts.
Floor joint models (interface z = 58, floor bottom against the seat top and the tongue), each a linear model; the real joint lies between the
bracket pairs (contact is unilateral, so a mode that lifts the floor sees something between the two):
  bonded       every interface node tied (upper bound, not physical)
  pre_hi       preloaded: nodes within BOLT_R / DOWEL_R of a bolt / dowel axis tied in all directions, and the rest of the interface closed in the
               normal direction (contact kept by the preload)
  pre_lo       preloaded: only the tied annuli; no contact anywhere else
  unpre_hi     unpreloaded: each M8 is an axial spring (E A_s / grip, spread over its 6 mm patch), dowels carry shear (x, y tied within 4 mm), the
               interface closed in the normal direction (contact), no friction
  unpre_lo     as unpre_hi with the contact open: bolt springs and dowels only
Seat supports: `plate` = the seat's +x face tied (all DOF) in 10 mm annuli round its 4 M8 + 2 dowels (the swivel plate's joint, treated
rigid); `bore` adds the 80 mm cartridge bore held rigid (stiff cartridge, upper bound).
"""
import json
import os
import sys
import numpy as np
import fe

BOLT_R, DOWEL_R = 12.0, 6.0
KN = 1e5  # contact normal penalty, N/mm per mm^2
BOLTS = [(-30.0, 135.0), (30.0, 135.0), (-30.0, 232.0), (30.0, 232.0)]  # fallback; the exporter's `cradle.bolts` replaces them (2026-11-06 design: front row at y 135)
DOWELS = [(-40.0, 190.0), (24.0, 190.0)]
SEAT_BOLTS = [(95.0, -38.0), (95.0, 46.0), (205.0, -38.0), (205.0, 46.0)]  # (y, z) at x = 45
SEAT_DOWELS = [(150.0, -35.0), (150.0, 35.0)]
FLANGE_HOLES = [(23.8198, 156.5635), (-39.8198, 156.5635), (-39.8198, 92.9239), (23.8198, 92.9239)]  # (x, z) at y = 239
WALL_FACE_Y = 239.0
FLOOR_Z = 58.0
BOLT_STIFFNESS = 210000.0 * 36.6 / 22.0  # M8 steel bolt, 22 mm grip, N/mm


def servo_group(export):
    """Mass (tonne), centre (authored frame) and inertia (tonne mm^2) of the servo group without the cradle."""
    members = [m for m in export["servo"]["members"] if m["id"] != "motorCradle"]
    shift = export["head"]["hob"][0]  # authored x = work x - 45.625 (work x of the hob centre): checked against the cradle's own centre
    shift = export["servo"]["cradleComWork"][0] - export["servo"]["cradleComAuthored"][0]
    mass = sum(m["mass"] for m in members)
    com = np.array([sum(m["mass"] * (m["com"][0] - shift) for m in members) / mass, sum(m["mass"] * m["com"][1] for m in members) / mass,
                    sum(m["mass"] * m["com"][2] for m in members) / mass])
    inertia = np.zeros((3, 3))
    for m in members:
        p = np.array([m["com"][0] - shift, m["com"][1], m["com"][2]]) - com
        inertia += m["mass"] * (np.dot(p, p) * np.eye(3) - np.outer(p, p))
    motor = next(m for m in members if m["id"] == "spindle/motor")
    body = export["servo"].get("motorBody", {"face": 90.0, "length": 115.0})
    a, b, c = body["face"], body["face"], body["length"]  # flange square, body length (the exporter's motor rating; the 2026-11-06 FE used 90 / 115)
    own = motor["mass"] / 12 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])  # body box about its own centre (x z flange, y long)
    own = motor["mass"] / 12 * np.diag([a * a + c * c, a * a + a * a, a * a + c * c])
    inertia += own
    return mass * 1e-3, com, inertia * 1e-3, shift


def face_triangles(conn, shared, nodes, z):
    """6-node triangles (corners + midpoints) on the plane z whose six nodes are all in `shared`; returns (corner triples, areas)."""
    tris = {}
    for e in conn:
        for f in ((0, 1, 2, 4, 5, 6), (0, 1, 3, 4, 9, 7), (1, 2, 3, 5, 8, 9), (0, 2, 3, 6, 8, 7)):
            ids = e[list(f)]
            if all(i in shared for i in ids) and np.all(np.abs(nodes[ids, 2] - z) < 1e-6):
                tris[tuple(sorted(ids[:3]))] = ids
    return list(tris.values())


def node_areas(nodes, tris):
    area = {}
    for ids in tris:
        p = nodes[ids[:3]]
        a = 0.5 * np.linalg.norm(np.cross(p[1] - p[0], p[2] - p[0]))
        for n in ids[3:]:
            area[n] = area.get(n, 0.0) + a / 3.0
    return area


def simple_seat_step(path):
    """Defeatured seat block: box x -48..45, y 70..222, z -50..58 with the 80 mm cartridge bore along y, plus the tongue (x -48..42, y 222..251, z 44..58).
    Tapped holes, dowel bores and the 2 mm clamp slit (closed by its three M6 bolts) are left out: the exported seat is 780.8 cm3 and this 0.8 % off."""
    gmsh = fe.import_gmsh()
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("seat")
        o = gmsh.model.occ
        block = o.addBox(-48, 70, -50, 93, 152, 108)
        tongue = o.addBox(-48, 222, 44, 90, 29, 14)
        bore = o.addCylinder(0, 69, 0, 0, 154, 0, 40)
        fused, _ = o.fuse([(3, block)], [(3, tongue)])
        o.cut(fused, [(3, bore)])
        o.synchronize()
        gmsh.write(path)
        return sum(o.getMass(3, t) for _, t in gmsh.model.getEntities(3))
    finally:
        gmsh.finalize()


class Cradle:
    def __init__(self, export, step_dir, size=8.0, seat_size=16.0, seat_support="plate", flange="bolts", curvature=4, rest=None):
        """rest: None (no body support), "rigid", or the stiffness in N/mm of the shim pack and its contact in series (z only, all rest-top nodes)."""
        self.export = export
        global BOLTS
        design = export.get("cradle")
        if design:
            BOLTS = [tuple(b) for b in design["bolts"]]
            if rest is not None:
                r = design["rest"]
                self.pad = {"box": [r["x0"], r["x1"], r["y0"], r["y1"]], "z": r["top"], "dirs": [2], "k": None if rest == "rigid" else float(rest)}
        seat_step = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"hobber-seat-simple-{os.getpid()}.step")
        self.seat_volume = simple_seat_step(seat_step)
        cradle_step = f"{step_dir}/GE-motor-cradle.step"
        part = next(p for p in export["parts"] if p["name"] == "GE-motor-cradle")
        floor_holes = [h for h in part["holes"] if h["direction"] == [0, 0, 1]]  # the floor's bolt, counterbore and dowel holes: filled (volume +0.5 %) so they do not drive the mesh
        if floor_holes:
            cradle_step = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"hobber-cradle-filled-{os.getpid()}.step")
            fe.filled_step(f"{step_dir}/GE-motor-cradle.step", floor_holes, cradle_step)
        pts, (cc, sc) = fe.mesh_parts([cradle_step, seat_step], seat_size, min_size=min(size / 2, 4.0), curvature=curvature,
                                      refine=[([-62, 39, 118, 248, 56, 175], size)])
        nodes, (cc, sc), pairs = fe.split_interface(pts, [cc, sc])
        self.nodes, self.cc, self.sc, self.pairs = nodes, cc, sc, pairs
        self.size, self.seat_support, self.flange = size, seat_support, flange
        self.mat_cradle = self.mat_seat = fe.STEEL  # diagnostics may stiffen one body (`dict(fe.STEEL, E=...)`) to see which one limits a mode
        self.vols = {}
        mass, com, inertia, shift = servo_group(export)
        self.servo = (mass, com, inertia)
        shared_c = {a for a, _ in pairs}
        tris = face_triangles(cc, shared_c, nodes, FLOOR_Z)
        self.area = node_areas(nodes, tris)
        self.pair_of = dict(pairs)

    def pad_selection(self, cp):
        (x0, x1, y0, y1), z = self.pad["box"], self.pad["z"]
        return (np.abs(cp[:, 2] - z) < 1e-6) & (cp[:, 0] >= x0) & (cp[:, 0] <= x1) & (cp[:, 1] >= y0) & (cp[:, 1] <= y1)

    def base_model(self, joint):
        """Model with supports, servo mass and the chosen floor-joint (`bonded`, `preloaded`, `unpreloaded`); returns Model and the contact pairs."""
        self.bolt_nodes = {}
        m = fe.Model()
        m.add_nodes(self.nodes)
        if getattr(self, "pad", None) and self.pad.get("k") is not None:  # dummy nodes carry the pad's series spring (added before the master)
            cn0 = fe.used_nodes(self.cc)
            ids = cn0[self.pad_selection(self.nodes[cn0])]
            start = m.add_nodes(self.nodes[ids])
            self.pad_dummies = list(range(start, start + len(ids)))
            for dm in self.pad_dummies:
                m.fix(dm)  # all three; the rigid constraint below overrides the directions the master carries
        m.add_body("cradle", self.cc, self.mat_cradle)
        m.add_body("seat", self.sc, self.mat_seat)
        nodes = self.nodes
        # Seat supports.
        seat_nodes = fe.used_nodes(self.sc)
        sp = nodes[seat_nodes]
        held = set()
        for y, z in SEAT_BOLTS:
            held |= set(seat_nodes[(np.abs(sp[:, 0] - 45) < 1e-6) & (np.hypot(sp[:, 1] - y, sp[:, 2] - z) <= 14)].tolist())
        for y, z in SEAT_DOWELS:
            held |= set(seat_nodes[(np.abs(sp[:, 0] - 45) < 1e-6) & (np.hypot(sp[:, 1] - y, sp[:, 2] - z) <= 10)].tolist())
        if self.seat_support == "face":  # the whole back face stuck to the saddle plate (friction of the preloaded bolts holds at these amplitudes)
            held |= set(seat_nodes[np.abs(sp[:, 0] - 45) < 1e-6].tolist())
        if self.seat_support == "bore":
            r = np.hypot(sp[:, 0], sp[:, 2])
            held |= set(seat_nodes[(np.abs(r - 40.0) < 0.6) & (sp[:, 1] > 70 - 1e-6) & (sp[:, 1] < 222 + 1e-6)].tolist())
        for n in held:
            m.fix(n)
        # Interface.
        for a, b in self.pairs:
            if b in held:
                continue
            x, y = nodes[a][0], nodes[a][1]
            near_bolt = any(np.hypot(x - bx, y - by) <= BOLT_R for bx, by in BOLTS)
            near_dowel = any(np.hypot(x - dx, y - dy) <= DOWEL_R for dx, dy in DOWELS)
            if joint == "bonded":
                m.tie(b, a)
            elif joint in ("pre_hi", "pre_lo"):
                if near_bolt or near_dowel:
                    m.tie(b, a)
                elif joint == "pre_hi":
                    m.tie(b, a, (2,))
            else:
                if any(np.hypot(x - dx, y - dy) <= 4.0 for dx, dy in DOWELS):
                    m.tie(b, a, (0, 1))
                patch = [(bx, by) for bx, by in BOLTS if np.hypot(x - bx, y - by) <= 6.0]
                if joint == "unpre_hi":
                    m.tie(b, a, (2,))
                elif patch:
                    self.bolt_nodes.setdefault(patch[0], []).append((a, b))
        for pt, lst in self.bolt_nodes.items():
            for a, b in lst:
                m.spring(3 * a + 2, 3 * b + 2, BOLT_STIFFNESS / len(lst))
        contact = []
        # Servo mass on the wall's front face.
        mass, com, inertia = self.servo
        mi = m.add_master("servo", com, mass, inertia)
        cn = fe.used_nodes(self.cc)
        cp = nodes[cn]
        face = np.abs(cp[:, 1] - WALL_FACE_Y) < 1e-6
        if self.flange == "bolts":
            sel = np.zeros(len(cn), bool)
            for x, z in FLANGE_HOLES:
                sel |= face & (np.hypot(cp[:, 0] - x, cp[:, 2] - z) <= 9.0)
        else:  # bonded flange: the whole flange footprint
            half = self.export["servo"].get("motorBody", {"face": 90.0})["face"] / 2
            sel = face & (np.abs(cp[:, 0] - (-8.0)) <= half) & (np.abs(cp[:, 2] - 124.7437) <= half)
        group = [int(n) for n in cn[sel]]
        m.rigid(mi, group)
        self.servo_master = mi
        self.servo_nodes = len(group)
        if getattr(self, "pad", None):  # a body support: the pad top coupled to the servo group in `dirs` (stuck contact / clamp bound)
            sel = self.pad_selection(cp)
            ids = [int(n) for n in cn[sel]]
            self.pad_nodes = len(ids)
            if self.pad.get("k") is None:
                m.rigid(mi, ids, tuple(self.pad["dirs"]))
            else:  # through a series spring (the jack screw's column), k in N/mm spread over the pad nodes, along each dir
                dummies = list(self.pad_dummies)
                m.rigid(mi, dummies, tuple(self.pad["dirs"]))
                for d in self.pad["dirs"]:
                    for a, b in zip(ids, dummies):
                        m.spring(3 * a + d, 3 * b + d, self.pad["k"] / len(ids))
        return m, contact

    def penalty(self, m, contact, active):
        for (a, b) in active:
            k = KN * self.area.get(a, 0.0)
            if k > 0:
                m.spring(3 * a + 2, 3 * b + 2, k)

    def classify(self, m, vec, M):
        """Effective-mass fractions along x, y, z and rotations, and the servo's displacement direction."""
        out = {}
        ph = np.asarray(vec)
        nd = len(ph)
        for k, name in enumerate("xyz"):
            r = np.zeros(nd)
            r[k:3 * m.nn:3] = 1.0
            for i in range(len(m.masters)):
                r[m.master_dof(i, k)] = 1.0
            num = (ph @ (M @ r)) ** 2
            out[name] = float(num / (ph @ (M @ ph)) / (r @ (M @ r)))
        d = np.array([ph[m.master_dof(self.servo_master, k)] for k in range(6)])
        out["servo_dir"] = (d[:3] / (np.linalg.norm(d[:3]) + 1e-30)).round(3).tolist()
        out["servo_rot"] = (d[3:] / (np.linalg.norm(d[:3]) + 1e-30)).round(5).tolist()
        return out

    def foot_stiffness(self, held="all"):
        """Cradle alone, its floor-bottom nodes held (`all` of them, or `bolts` = only the bolt/dowel annuli): 6 x 6 stiffness at the servo's centre
        through the same flange coupling, to compare with the hand model's wall / vertical / lateral springs (each frequency = sqrt(k / m) / 2 pi
        with the servo group's mass, ignoring coupling between the directions)."""
        nodes = self.nodes
        m = fe.Model()
        m.add_nodes(nodes)
        m.add_body("cradle", self.cc, fe.STEEL)
        used = set(fe.used_nodes(self.cc).tolist())
        for n in range(len(nodes)):
            if n not in used:
                m.fix(n)
        cn = fe.used_nodes(self.cc)
        for a, b in self.pairs:
            x, y = nodes[a][0], nodes[a][1]
            ok = held == "all" or any(np.hypot(x - bx, y - by) <= BOLT_R for bx, by in BOLTS) or any(np.hypot(x - dx, y - dy) <= DOWEL_R for dx, dy in DOWELS)
            if ok:
                m.fix(a)
        mass, com, inertia = self.servo
        mi = m.add_master("servo", com, mass, inertia)
        cp = nodes[cn]
        face = np.abs(cp[:, 1] - WALL_FACE_Y) < 1e-6
        if self.flange == "bolts":
            sel = np.zeros(len(cn), bool)
            for x, z in FLANGE_HOLES:
                sel |= face & (np.hypot(cp[:, 0] - x, cp[:, 2] - z) <= 9.0)
        else:
            half = self.export["servo"].get("motorBody", {"face": 90.0})["face"] / 2
            sel = face & (np.abs(cp[:, 0] - (-8.0)) <= half) & (np.abs(cp[:, 2] - 124.7437) <= half)
        m.rigid(mi, [int(n) for n in cn[sel]])
        K, M, _ = m.system()
        T, free = m.reduction()
        solve = m.static_solver(K, T, free, "direct")
        C = np.zeros((6, 6))
        for k in range(6):
            u = solve({m.master_dof(mi, k): 1.0})
            C[:, k] = [u[m.master_dof(mi, j)] for j in range(6)]
        Km = np.linalg.inv(C)
        Mm = np.zeros((6, 6))
        Mm[:3, :3] = mass * np.eye(3)
        Mm[3:, 3:] = inertia
        import scipy.linalg as sl
        w2 = sl.eigh(0.5 * (Km + Km.T), Mm, eigvals_only=True)
        rigid_foot_modes = [float(np.sqrt(abs(v)) / (2 * np.pi)) for v in w2[:3]]
        kk = {"x": Km[0, 0], "y": Km[1, 1], "z": Km[2, 2]}
        return {"held": held, "flange": self.flange, "k_N_per_um": {a: v / 1000 for a, v in kk.items()},
                "f_Hz": {a: float(np.sqrt(v * 1000 / (mass * 1e3)) / (2 * np.pi)) for a, v in kk.items()},
                "compliance_direct_f_Hz": {a: float(np.sqrt(1 / C[i, i] * 1000 / (mass * 1e3)) / (2 * np.pi)) for i, a in enumerate("xyz")},
                "first_modes_rigid_foot_Hz": rigid_foot_modes, "servo_mass_kg": mass * 1e3, "servo_com": com.tolist(), "rigid_nodes": int(sel.sum())}

    def modes(self, joint, count=4):
        m, _ = self.base_model(joint)
        K, M, vols = m.system()
        T, _ = m.reduction()
        f, V = m.modes(count, K, M, T, method="direct")
        return {"joint": joint, "frequencies_Hz": [float(x) for x in f], "volumes": vols, "dofs": int(T.shape[1]), "servo_rigid_nodes": self.servo_nodes,
                "modes": [self.classify(m, V[:, i], M) | {"f": float(f[i])} for i in range(count)]}


JOINTS = ("bonded", "pre_hi", "pre_lo", "unpre_hi", "unpre_lo")


def one(args):
    export_dir, size, seat_size, cur, support, joint, flange = args[:7]
    rest = args[7] if len(args) > 7 else None
    export = json.load(open(f"{export_dir}/hobber-fem-export.json"))
    c = Cradle(export, export_dir, size, seat_size, support, flange, cur, rest)
    r = c.modes(joint, 4)
    r.update(support=support, joint=joint, flange=flange, size=size, seat_size=seat_size, rest=rest, elements=[len(c.cc), len(c.sc)])
    return r


def run(jobs, workers=3):
    """jobs: list of (export_dir, size, seat_size, curvature, support, joint, flange)."""
    from multiprocessing import Pool
    with Pool(workers) as p:
        return p.map(one, jobs, chunksize=1)


def foot(args):
    export_dir, size, seat_size, cur, held, flange = args
    export = json.load(open(f"{export_dir}/hobber-fem-export.json"))
    c = Cradle(export, export_dir, size, seat_size, "plate", flange, cur)
    r = c.foot_stiffness(held)
    r.update(size=size, elements=len(c.cc))
    return r


def plan(export_dir, name):
    if name == "final":  # the 2026-11-06 design: rest block carrying the servo body through a shim pack (3000 N/um assumed; a jack screw pair 600 N/um; none for the effect)
        j = [(export_dir, 8.0, 16.0, 4, "plate", jt, fl, 3e6) for fl in ("bolts", "face") for jt in JOINTS]
        j += [(export_dir, 8.0, 16.0, 4, "plate", jt, "bolts", 6e5) for jt in ("pre_hi", "pre_lo", "unpre_hi")]
        j += [(export_dir, 8.0, 16.0, 4, "plate", jt, "bolts", None) for jt in JOINTS]
        j += [(export_dir, 8.0, 16.0, 4, "bore", jt, "face", 3e6) for jt in ("pre_hi", "pre_lo")]
        return j
    if name == "final-fine":  # mesh check of the final design: first mode at 6 mm (cradle) / 14 mm (seat), preloaded
        return [(export_dir, 6.0, 14.0, 4, "plate", jt, "bolts", 3e6) for jt in ("pre_hi", "pre_lo")]
    if name == "medium":
        j = [(export_dir, 8.0, 16.0, 4, "plate", jt, fl) for fl in ("bolts", "face") for jt in JOINTS]
        j += [(export_dir, 8.0, 16.0, 4, "bore", jt, "face") for jt in ("pre_hi", "pre_lo")]
        return j
    if name == "foot":
        return [(export_dir, 8.0, 16.0, 4, h, fl) for h in ("all", "bolts") for fl in ("bolts", "face")]
    if name == "fine":
        return [(export_dir, 5.0, 14.0, 4, "plate", jt, fl) for fl in ("bolts", "face") for jt in ("pre_hi", "pre_lo")]
    raise ValueError(name)


if __name__ == "__main__":
    d, out, name = sys.argv[1], sys.argv[2], sys.argv[3]
    if name == "foot":
        from multiprocessing import Pool
        with Pool(3) as p:
            results = p.map(foot, plan(d, name), chunksize=1)
        json.dump(results, open(out, "w"), indent=1)
        for r in results:
            print("foot", r["held"], r["flange"], [round(x) for x in r["first_modes_rigid_foot_Hz"]], {k: round(v, 1) for k, v in r["k_N_per_um"].items()}, {k: round(v) for k, v in r["f_Hz"].items()}, flush=True)
        sys.exit(0)
    results = run(plan(d, name))
    json.dump(results, open(out, "w"), indent=1)
    for r in results:
        print(r["size"], r["support"], r["flange"], r["joint"], r.get("rest"), [round(x) for x in r["frequencies_Hz"][:4]], r["dofs"], flush=True)
