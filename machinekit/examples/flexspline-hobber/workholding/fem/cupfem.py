"""Structured hex mesher + CalculiX driver for the flexspline cup on its workholding.

Units N, mm, MPa.  Half model (theta 0..pi, plane y=0) with symmetric or antisymmetric plane BCs.
Bodies (all optional except the cup):
  cup    r 29.025..29.625 (bore 58.05, root OD 59.25 smeared ring), z 0..40 (hub plate omitted, see README)
  sleeve thin tube z 0..Ls under the tooth band, end lands rigidly fixed, oil pressure on the free inner face
  plug   solid-ish tube (r 5..rp) z 0..18, base fixed
  land   0.3 mm stop land block z -1..0, base fixed
"""
import os, subprocess, re, math, shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../../../.."))
CCX = REPO + "/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
LIB = REPO + "/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu"
HEAVY = "/home/joao/dev/materia-cache/tools/heavy-run"
WORK = os.environ.get("CUPFEM_WORK", HERE + "/work")

E, NU = 210000.0, 0.3
RI, RO, H, BAND = 29.025, 29.625, 40.0, 8.0     # bore r, root r (wall 0.6), cup height, tooth band
LAND_W = 0.3                                     # stop land radial width (dia 58.65)


def zgrid(fine=1.0):
    """axial node positions of the cup; `fine` scales element size (0.5 = refined)."""
    z = list(np.arange(0, 16 + 1e-9, 0.5 * fine))
    z += list(np.arange(16, 24 + 1e-9, 1.0 * fine))[1:]
    z += list(np.arange(24, 40 + 1e-9, 2.0 * fine))[1:]
    return np.array(z)


class Mesh:
    def __init__(self):
        self.xyz = []        # node coords
        self.elems = []      # (id, [8 nodes], elset)
        self.blocks = {}

    def block(self, name, rfun, thetas, zs, nr, elset, outer=None):
        """rfun(theta)->(r_in, r_out); nr radial elements. node index array idx[ir,it,iz]."""
        nt, nz = len(thetas), len(zs)
        idx = -np.ones((nr + 1, nt, nz), int)
        for it, th in enumerate(thetas):
            r0, r1 = rfun(th)
            for ir in range(nr + 1):
                if outer is not None and ir == nr:
                    for iz in range(nz): idx[ir, it, iz] = outer[it, iz]
                    continue
                r = r0 + (r1 - r0) * ir / nr
                for iz, z in enumerate(zs):
                    x, y = r * math.cos(th), r * math.sin(th)
                    if abs(th - math.pi) < 1e-9 or th == 0: y = 0.0
                    self.xyz.append((x, y, z)); idx[ir, it, iz] = len(self.xyz)
        eid = np.zeros((nr, nt - 1, nz - 1), int)
        for ir in range(nr):
            for it in range(nt - 1):
                for iz in range(nz - 1):
                    n = [idx[ir, it, iz], idx[ir + 1, it, iz], idx[ir + 1, it + 1, iz], idx[ir, it + 1, iz],
                         idx[ir, it, iz + 1], idx[ir + 1, it, iz + 1], idx[ir + 1, it + 1, iz + 1], idx[ir, it + 1, iz + 1]]
                    self.elems.append((len(self.elems) + 1, n, elset)); eid[ir, it, iz] = len(self.elems)
        self.blocks[name] = dict(idx=idx, eid=eid, thetas=np.array(thetas), zs=np.array(zs), nr=nr)
        return self.blocks[name]


def faces(b, which, zlo=-1e9, zhi=1e9):
    """element-face list (eid, S#) for a block face: 'in'(S6) 'out'(S4) 'bot'(S1) 'top'(S2); elements with z-centre in range."""
    s = dict(inn=6, out=4, bot=1, top=2)[which]
    nr, nt, nz = b["eid"].shape
    out = []
    zc = 0.5 * (b["zs"][:-1] + b["zs"][1:])
    for iz in range(nz):
        if not (zlo <= zc[iz] <= zhi): continue
        for it in range(nt):
            if which == "inn": out.append((b["eid"][0, it, iz], s))
            elif which == "out": out.append((b["eid"][nr - 1, it, iz], s))
            elif which == "bot":
                if iz == 0:
                    out += [(b["eid"][ir, it, 0], s) for ir in range(nr)]
            elif which == "top":
                if iz == nz - 1:
                    out += [(b["eid"][ir, it, nz - 1], s) for ir in range(nr)]
    return out


def run_model(name, spec):
    """spec keys: nth, fine, p_bore (uniform bore pressure MPa, z range pz), sleeve{...}, plug{rp}, land_mode, loads..., returns dict."""
    d = os.path.join(WORK, name); os.makedirs(d, exist_ok=True)
    nth = spec.get("nth", 36)
    th = np.linspace(0, math.pi, nth + 1)
    m = Mesh()
    zs = zgrid(spec.get("fine", 1.0))
    if spec.get("height") is not None: zs = zs[zs <= spec["height"] + 1e-9]
    nrc = spec.get("nr_cup", 2)
    cup = m.block("cup", lambda t: (RI, RO), th, zs, nrc, "CUP")
    sl = pl = ld = None
    if "sleeve" in spec:
        s = spec["sleeve"]; g = s.get("gap", 0.01); t0 = s.get("wall", 1.5); Ls = s.get("length", 16.0)
        n, a = s.get("wall_n", 0), s.get("wall_a", 0.0)         # wall = t0 + a cos(n th)
        bn, b = s.get("od_n", 0), s.get("od_b", 0.0)            # OD = RI-g + b cos(n th)
        zsl = zs[zs <= Ls + 1e-9]
        def rf(t):
            ro = RI - g + b * math.cos(bn * t)
            return ro - t0 - a * math.cos(n * t), ro
        sl = m.block("sleeve", rf, th, zsl, 3, "SLV", outer=cup["idx"][0, :, :len(zsl)] if s.get("tie") else None)
    if "plug" in spec:
        rp = RI - spec["plug"]["clear_r"]
        zp = zs[zs <= spec["plug"].get("length", 18.0) + 1e-9]
        pl = m.block("plug", lambda t: (5.0, rp), th, zp, 3, "PLUG")
        sx = spec["plug"].get("shift_x", 0.0)
        for i in set(pl["idx"].ravel()):
            x, y, z = m.xyz[i - 1]; m.xyz[i - 1] = (x + sx, y, z)
    if spec.get("land_contact"):
        ld = m.block("land", lambda t: (RI, RI + LAND_W), th, np.array([-1.0, 0.0]), 2, "LAND")
    X = np.array(m.xyz)
    used = set()
    for _, n, _ in m.elems: used.update(n)
    I = cup["idx"]; L = []; a = L.append
    a("*NODE")
    for i, (x, y, z) in enumerate(m.xyz): a(f"{i+1},{x:.9g},{y:.9g},{z:.9g}")
    for es in ["CUP", "SLV", "PLUG", "LAND"]:
        el = [e for e in m.elems if e[2] == es]
        if el:
            a(f"*ELEMENT,TYPE=C3D8I,ELSET={es}")
            for eid, n, _ in el: a(f"{eid}," + ",".join(map(str, n)))
    a("*MATERIAL,NAME=ST\n*ELASTIC\n%g,%g" % (E, NU))
    for es in ["CUP", "SLV", "PLUG", "LAND"]:
        if any(e[2] == es for e in m.elems): a(f"*SOLID SECTION,ELSET={es},MATERIAL=ST")

    def nset(nm, ids):
        ids = sorted(set(int(i) for i in ids)); a(f"*NSET,NSET={nm}")
        for k in range(0, len(ids), 12): a(",".join(map(str, ids[k:k + 12])) + ",")
    def elset_faces(nm, fl):
        a(f"*SURFACE,NAME={nm},TYPE=ELEMENT")
        for e, s in fl: a(f"{e},S{s}")

    # --- boundary conditions
    sym = spec.get("sym", "S")                 # S: uy=0 on y=0 ; A: ux=uz=0 on y=0
    plane = [i for i in range(1, len(X) + 1) if abs(X[i - 1][1]) < 1e-12 and i in used]
    if sym == "S": nset("PLANE", plane); a("*BOUNDARY\nPLANE,2,2")
    else: nset("PLANE", plane); a("*BOUNDARY\nPLANE,1,1\nPLANE,3,3")
    th_of = lambda i: math.atan2(X[i - 1][1], X[i - 1][0]) if abs(X[i-1][1]) > 1e-12 else (0.0 if X[i-1][0] > 0 else math.pi)
    eqs = []
    # stop land: cup bottom nodes on the land
    landn = [I[ir, it, 0] for ir in range(I.shape[0]) for it in range(I.shape[1]) if RI + LAND_W * (ir / nrc * 0 ) <= 1e99 and
             math.hypot(*X[I[ir, it, 0] - 1][:2]) <= RI + LAND_W + 1e-6]
    nset("LANDN", landn)
    lm = spec.get("land_mode", "slide")        # slide: uz only (rim free to slide radially) ; stuck: ux=uy=0 too (friction holds)
    if ld is None:
        a("*BOUNDARY\nLANDN,3,3")
    else:
        # land block fixed at base; contact pair (cup bottom, land top) frictionless
        nset("LANDBASE", ld["idx"][:, :, 0].ravel()); a("*BOUNDARY\nLANDBASE,1,3")
    if lm == "stuck": a("*BOUNDARY\nLANDN,1,2")
    if sl is not None:
        sI = sl["idx"]; llo, lhi = spec["sleeve"].get("land_lo", 1.5), spec["sleeve"].get("land_hi", 1.5)
        zsl = sl["zs"]; Ls = zsl[-1]
        fixed = [sI[0, it, iz] for it in range(sI.shape[1]) for iz in range(len(zsl)) if (llo > 0 and zsl[iz] <= llo + 1e-9) or zsl[iz] >= Ls - lhi - 1e-9]
        nset("SLVFIX", fixed); a("*BOUNDARY\nSLVFIX,1,3")
        if spec["sleeve"].get("rigid_back"):
            nset("SLVBACK", sI[0].ravel()); a("*BOUNDARY\nSLVBACK,1,3")
    if pl is not None:
        nset("PLUGBASE", pl["idx"][:, :, 0].ravel()); a("*BOUNDARY\nPLUGBASE,1,3")
    if spec.get("pin_x"):                      # remove x-translation of an axisymmetric slide-mode cup
        nset("PINX", [I[nrc, nth // 2, -1], I[0, nth // 2, -1]]); a("*BOUNDARY\nPINX,1,1")
    if spec.get("fix_top_radial"):
        nset("TOPR", I[:, :, -1].ravel()); a("*BOUNDARY\nTOPR,1,2")
    # soft springs against rigid-body drift in contact models
    sp = spec.get("soft_spring")
    if sp:
        i0 = I[1, 0, -1]; a(f"*ELEMENT,TYPE=SPRING1,ELSET=SPR\n{len(m.elems)+1},{i0}\n*SPRING,ELSET=SPR\n{sp[0]}\n{sp[1]:g}")
    # --- contact
    contact = []
    if sl is not None:
        elset_faces("CUPBORE", faces(cup, "inn", 0, sl["zs"][-1])); elset_faces("SLVOD", faces(sl, "out"))
        if not spec["sleeve"].get("tie"): contact.append(("CUPBORE", "SLVOD"))
    if pl is not None:
        elset_faces("CUPBORE2", faces(cup, "inn", 0, pl["zs"][-1])); elset_faces("PLUGOD", faces(pl, "out"))
        contact.append(("CUPBORE2", "PLUGOD"))
    if ld is not None:
        elset_faces("CUPBOT", [f for f in faces(cup, "bot") if f[0] in set(cup["eid"][:2, :, 0].ravel())])
        elset_faces("LANDTOP", faces(ld, "top")); contact.append(("CUPBOT", "LANDTOP"))
    K = spec.get("K", 1e5)
    if contact:
        a(f"*SURFACE INTERACTION,NAME=IC\n*SURFACE BEHAVIOR,PRESSURE-OVERCLOSURE=LINEAR\n{K:g}")
        for sv, ms in contact: a(f"*CONTACT PAIR,INTERACTION=IC,TYPE=SURFACE TO SURFACE\n{sv},{ms}")
    # --- output sets
    out_ids = list(I[nrc, :, :].ravel())
    nset("OUTER", out_ids); nset("BOREN", I[0].ravel())
    # --- step
    nl = bool(contact)
    a("*STEP,INC=200" if nl else "*STEP")
    a("*STATIC\n0.1,1,1e-4,1" if nl else "*STATIC")
    if "p_bore" in spec:
        p, z0, z1 = spec["p_bore"]
        a("*DLOAD")
        for e, s in faces(cup, "inn", z0, z1): a(f"{e},P{s},{p:g}")
    if sl is not None and spec["sleeve"].get("oil", 0) != 0:
        llo, lhi = spec["sleeve"].get("land_lo", 1.5), spec["sleeve"].get("land_hi", 1.5)
        a("*DLOAD")
        for e, s in faces(sl, "inn", llo, sl["zs"][-1] - lhi): a(f"{e},P{s},{spec['sleeve']['oil']:g}")
    cl = []
    def patch_nodes(zmax, thp):
        ids, w = [], []
        for iz, z in enumerate(cup["zs"]):
            if z > zmax + 1e-9: continue
            for it, t in enumerate(th):
                if t > thp + 1e-9: continue
                wz = 0.5 if (iz == 0 or abs(z - zmax) < 1e-9) else 1.0
                wt = 0.5 if it == 0 else 1.0
                ids.append(I[nrc, it, iz]); w.append(wz * wt)
        w = np.array(w); return ids, w / w.sum()
    ld_spec = spec.get("force")                # dict(kind='rad'|'tan', F=10, thp=deg)
    if ld_spec:
        ids, w = patch_nodes(BAND, math.radians(ld_spec.get("thp", 5)))
        F = ld_spec["F"] / 2.0              # half model
        a("*CLOAD")
        for i, wi in zip(ids, w):
            if ld_spec["kind"] == "rad": a(f"{i},1,{-F*wi:.9g}")
            else: a(f"{i},2,{F*wi:.9g}")
    ax = spec.get("axial")                    # downward total force N through the top edge
    if ax:
        ids = I[:, :, -1]
        a("*CLOAD")
        wt = np.where((np.arange(ids.shape[1]) == 0) | (np.arange(ids.shape[1]) == ids.shape[1] - 1), 0.5, 1.0)
        tot = ids.shape[0] * 0 + 0
        W = np.outer(np.ones(ids.shape[0]), wt); W[[0, -1], :] *= 0.5; W /= W.sum()
        for ir in range(ids.shape[0]):
            for it in range(ids.shape[1]): a(f"{ids[ir,it]},3,{-ax/2.0*W[ir,it]:.9g}")
    a("*NODE PRINT,NSET=OUTER\nU")
    if sl is not None:
        nset("SLVOUT", sl["idx"][-1].ravel()); a("*NODE PRINT,NSET=SLVOUT\nU")
    a("*END STEP")
    with open(f"{d}/m.inp", "w") as f: f.write("\n".join(L) + "\n")
    env = dict(os.environ, LD_LIBRARY_PATH=LIB + ":" + os.environ.get("LD_LIBRARY_PATH", ""), OMP_NUM_THREADS="4")
    cmd = [HEAVY, "--mem", "6G", "--need", "3G", "--slots", "4", "--label", "cupfem-" + name, "--", CCX, "m"]
    r = subprocess.run(cmd, cwd=d, env=env, capture_output=True, text=True)
    open(f"{d}/run.log", "w").write(r.stdout + r.stderr)
    if not os.path.exists(f"{d}/m.dat") or "*ERROR" in r.stdout:
        raise RuntimeError(f"ccx failed for {name}; see {d}/run.log\n" + r.stdout[-1500:])
    return parse(d, X, m, cup, sl)


def parse(d, X, m, cup, sl):
    txt = open(f"{d}/m.dat").read()
    sets, S, cur = {}, {}, None
    for line in txt.splitlines():
        if "displacements" in line:
            nm = line.split("for set")[1].split()[0]; cur = sets[nm] = {}; continue
        if not line.strip(): continue
        p = line.split()
        try:
            vals = [float(v) for v in p]
        except ValueError:
            cur = None; continue
        if cur is not None: cur[int(vals[0])] = vals[1:]
    U = sets["OUTER"]
    I = cup["idx"]; nrc = I.shape[0] - 1
    th, zs = cup["thetas"], cup["zs"]
    ur = np.zeros((len(th), len(zs))); ut = np.zeros_like(ur); uz = np.zeros_like(ur)
    for it, t in enumerate(th):
        for iz in range(len(zs)):
            u = U[int(I[nrc, it, iz])]
            ur[it, iz] = u[0] * math.cos(t) + u[1] * math.sin(t)
            ut[it, iz] = -u[0] * math.sin(t) + u[1] * math.cos(t)
            uz[it, iz] = u[2]
    res = dict(th=th, zs=zs, ur=ur, ut=ut, uz=uz, ux=None)
    if sl is not None:
        SI = sl["idx"]; V = sets["SLVOUT"]; nz = SI.shape[2]
        sr = np.zeros((len(th), nz))
        for it, t in enumerate(th):
            for iz in range(nz):
                u = V[int(SI[-1, it, iz])]; sr[it, iz] = u[0] * math.cos(t) + u[1] * math.sin(t)
        res["sl_ur"] = sr; res["sl_zs"] = sl["zs"]
    return res


def band_stats(res, zmax=BAND, col="ur"):
    """mean over band & circumference, taper (max-min of circumferential mean along z), circumferential variation."""
    zs = res["zs"]; sel = zs <= zmax + 1e-9
    a = res[col][:, sel]
    th = res["th"]
    w = np.ones(len(th)); w[0] = w[-1] = 0.5; w /= w.sum()
    zmean = (w[:, None] * a).sum(0)            # circumferential mean vs z
    return dict(mean=float(zmean.mean()), taper=float(zmean.max() - zmean.min()),
                z0=float(zmean[0]), zend=float(zmean[-1]), zmean=zmean, zs=zs[sel],
                circ_pp=float((a.max(1) - a.min(1)).max()), circ_over_band=a.mean(1))


def harmonics(th, f, nmax=4):
    """cos-harmonic amplitudes of f(theta) on half circle theta 0..pi (symmetric about plane): returns a_n."""
    # f is even about theta=0, so f = sum a_n cos(n th); least squares on the nodes
    A = np.array([[math.cos(n * t) for n in range(nmax + 1)] for t in th])
    c, *_ = np.linalg.lstsq(A, f, rcond=None)
    return c
