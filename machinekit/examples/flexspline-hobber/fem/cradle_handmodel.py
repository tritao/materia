"""Hand model of the B servo group on its cradle: the Python twin of `HobberFrameChecks.spindleDrive`, and the fit of its constants to the cradle FE.

Two coupled-pair modes, each a 2 x 2 generalised eigenproblem with the servo's mass and inertia:
  rocking  the wall bends as a cantilever (the section at each height: the wall at full thickness, the rib at a fitted participation `eta`) from the foot's top to the upper
           flange bolt row, where the servo is attached; the base turns about x (`kbx`: floor joint and seat); the rest block adds a vertical spring ahead of the wall
           (shim pack in series with the seat's local stiffness `kpb`).   DOFs: y displacement and rotation about x of the attachment point.
  lateral  the same cantilever bending in x, the base rolling about y (`kby`), the rest block's width as a roll spring.
Usage: python3 cradle_handmodel.py <export dir> [fit]   prints the model against results/cradle-variants.json (and refits kpb, kby, eta, kbx with `fit`).
The constants in use are `HobberFeResults.hx`; paste a refit there.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import scipy.linalg as sl

HERE = Path(__file__).resolve().parent
E_, G_ = 210000.0, 210000.0 / 2.6
C, FACE, ZQ, ZB = 124.7437, 239.0, 156.5635, 76.0  # motor axis height, wall face y, upper flange bolt row, foot top (authored frame, mm)
CONST = dict(kbx=1.31e9, kby=2.23e8, eta=0.19, kpb=3.5e5)


def slabs(z, p, with_rib):
    t = p.get("wall", 8.0)
    rear = FACE + t
    rects = [(-56.0, 31.0, FACE, rear)]
    if with_rib and p.get("rib", True):
        rx0 = p.get("ribx") if p.get("ribx") is not None else -61.0
        rt = p.get("ribt", 6.0)
        if p.get("window", True) and 95.0 <= z <= 160.0:
            rects += [(rx0, rx0 + rt, 192.0, 209.0), (rx0, rx0 + rt, 232.0, rear)]
        else:
            rects.append((rx0, rx0 + rt, 192.0, rear))
    xs = sorted({r[0] for r in rects} | {r[1] for r in rects})
    out = []
    for a, b in zip(xs[:-1], xs[1:]):
        cov = [r for r in rects if r[0] <= 0.5 * (a + b) <= r[1]]
        if cov:
            out.append((a, b, min(r[2] for r in cov), max(r[3] for r in cov)))
    return out


def section(z, p, with_rib, axis):
    sl_ = slabs(z, p, with_rib)
    area = sum((b - a) * (y1 - y0) for a, b, y0, y1 in sl_)
    mid = lambda a, b, y0, y1: 0.5 * (y0 + y1) if axis == "x" else 0.5 * (a + b)
    c = sum((b - a) * (y1 - y0) * mid(a, b, y0, y1) for a, b, y0, y1 in sl_) / area
    inertia = sum(((b - a) * (y1 - y0) ** 3 if axis == "x" else (y1 - y0) * (b - a) ** 3) / 12 + (b - a) * (y1 - y0) * (mid(a, b, y0, y1) - c) ** 2 for a, b, y0, y1 in sl_)
    return area, inertia


def servo(export):
    s = export["servo"]
    members = [m for m in s["members"] if m["id"] != "motorCradle"]
    shift = s["cradleComWork"][0] - s["cradleComAuthored"][0]
    mass = sum(m["mass"] for m in members)
    com = np.array([sum(m["mass"] * (m["com"][i] - (shift if i == 0 else 0)) for m in members) / mass for i in range(3)])
    body = s.get("motorBody", {"face": 90.0, "length": 115.0})
    motor = next(m for m in members if m["id"] == "spindle/motor")
    ixx = sum(m["mass"] * ((m["com"][1] - com[1]) ** 2 + (m["com"][2] - com[2]) ** 2) for m in members) + motor["mass"] / 12 * (body["face"] ** 2 + body["length"] ** 2)
    iyy = sum(m["mass"] * ((m["com"][0] - shift - com[0]) ** 2 + (m["com"][2] - com[2]) ** 2) for m in members) + motor["mass"] / 12 * 2 * body["face"] ** 2
    return mass, com, ixx, iyy


def flex(p, eta, axis, steps=None):
    h = ZQ - ZB
    n = int(round(h))
    f = np.zeros(3)
    for i in range(n):
        s = (i + 0.5) * h / n
        a1, i1 = section(ZB + s, p, True, axis)
        a0, i0 = section(ZB + s, p, False, axis)
        a, inertia = a0 + eta * (a1 - a0), i0 + eta * (i1 - i0)
        arm, ds = h - s, h / n
        f += np.array([arm * arm / (E_ * inertia) + 1 / (5 / 6 * G_ * a), arm / (E_ * inertia), 1 / (E_ * inertia)]) * ds
    return f, h


def modes(p, rest, mdl, const=CONST, pack=3.0e6):
    mass, com, ixx, iyy = mdl
    mt = mass * 1e-3
    ry, rz = com[1] - FACE, com[2] - ZQ
    out = {}
    for axis, kb, name in (("x", const["kbx"], "rocking"), ("y", const["kby"], "lateral")):
        f, h = flex(p, const["eta"], axis)
        K = np.linalg.inv(np.array([[f[0] + h * h / kb, f[1] + h / kb], [f[1] + h / kb, f[2] + 1 / kb]]))
        if rest:
            k = rest["k"] if rest.get("k") else pack
            ke = 1 / (1 / k + 1 / const["kpb"])
            if axis == "x":
                K[1, 1] += ke * (FACE - 0.5 * (rest["box"][2] + rest["box"][3])) ** 2
            else:
                w = rest["box"][1] - rest["box"][0]
                K[1, 1] += ke * (w * w / 12 + (0.5 * (rest["box"][0] + rest["box"][1]) + 8.0) ** 2)
        if axis == "x":
            T = np.array([[1.0, -rz], [0.0, ry]])
            Mm = mt * T.T @ T
            Mm[1, 1] += ixx * 1e-3
        else:
            T = np.array([[1.0, rz]])
            Mm = mt * T.T @ T
            Mm[1, 1] += iyy * 1e-3
        out[name] = float(np.sqrt(sl.eigh(K, Mm, eigvals_only=True)[0]) / 2 / math.pi)
    return out


def variants():
    rows = json.load(open(HERE / "results" / "cradle-variants.json"))["variants"]
    for r in rows:
        rest = r["rest"]
        if rest:
            rest = {"k": rest["k"], "box": [rest["x0"], rest["x1"], rest["y0"], rest["y1"]]}
        yield r["name"], r["params"], rest, r["FE_Hz"]


if __name__ == "__main__":
    export = json.load(open(Path(sys.argv[1]) / "hobber-fem-export.json"))
    mdl = servo(export)
    print("servo", round(mdl[0], 3), "kg, com", mdl[1].round(2), "Ixx", round(mdl[2]), "Iyy", round(mdl[3]))
    const = dict(CONST)
    if "fit" in sys.argv[2:]:
        from scipy.optimize import least_squares
        pad = [v for v in variants() if v[2]]

        def res(x):
            c = dict(const, kpb=math.exp(x[0]), kby=math.exp(x[1]))
            return [math.log(min(modes(p, r, mdl, c).values()) / fe_[0]) for n, p, r, fe_ in pad]
        s = least_squares(res, [math.log(const["kpb"]), math.log(const["kby"])], bounds=([math.log(1e4), math.log(1e7)], [math.log(1e9), math.log(1e12)]))
        const.update(kpb=math.exp(s.x[0]), kby=math.exp(s.x[1]))
        print("refit", {k: float("%.3g" % v) for k, v in const.items()})
    worst = 0.0
    for n, p, r, fe_ in variants():
        m = modes(p, r, mdl, const)
        first = min(m.values())
        print(f"{n:6s} FE {fe_[0]:4d} / {fe_[1]:4d}   hand rocking {m['rocking']:6.0f} lateral {m['lateral']:6.0f}  first {first:5.0f} ({100 * (first / fe_[0] - 1):+4.0f} %)")
        if r:
            worst = max(worst, abs(first / fe_[0] - 1))
    print("rest-block variants: worst first-mode error %.0f %%" % (100 * worst))
