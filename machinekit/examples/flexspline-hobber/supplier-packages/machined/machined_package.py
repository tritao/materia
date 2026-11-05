#!/usr/bin/env python3
"""Drawings, summary, RFQ table and checks for the hobber's machined and ground-stock parts.

Reads <dir>/machined-package.json and <dir>/step/*.step (written by MachinedPackageExport.hx) and writes into <dir>:
  parts.json                 every part: id, route, stock, material, mass, size, quantity, holes, faces, checks
  PARTS_RFQ.md, parts-rfq.csv  one row per part: process, who it goes to, stock, size, mass, holes, tolerances
  drawings/<id>.html/.pdf    one A4 landscape sheet per part: third-angle views with hidden hole edges, overall
                             dimensions, part data, machined-face table, hole table, notes, title block
  drawings/<id>.dxf          the three views 1:1 (R12, mm; layers VISIBLE_<VIEW>, HIDDEN_<VIEW>, CENTRE, DIM, TEXT)

Usage: machined_package.py <dir>

Everything on a sheet comes from the model (route, stock row, holes, mass); this script adds only the ordering
requirements the model does not hold, listed in ORDERING below. It stops (exit 1) when numbers and parts disagree:
STEP re-import volume vs model volume (0.1 %), a table hole missing from the solid, the milled-part rules
(at most 2 setups, at most 600 mm), stock thickness vs the part, the drawings read back against the data.
"""
import csv
import html
import json
import math
import os
import shutil
import subprocess
import sys
from datetime import date

import numpy as np

COMPANY = "Materia Expedita, Rua das Oficinas Municipais 15, 2450-060 Nazare, Portugal"
CONTACT = "joao@materia-expedita.pt"
REV = "A"
VOLUME_TOLERANCE = 1e-3
# ISO 2768 general tolerances on linear dimensions (mm, +/-): range upper bound -> value.
ISO_2768 = {
    "m": [(6, 0.1), (30, 0.2), (120, 0.3), (400, 0.5), (1000, 0.8)],
    "f": [(6, 0.05), (30, 0.1), (120, 0.15), (400, 0.2), (1000, 0.3)],
}
CLASS_OF_ROUTE = {"general": "m", "fine": "f", "tight": "f"}
# Ordering requirements the model does not hold (to confirm with the supplier).
ORDERING = [
    "Material grade and heat treatment are not in the model (generic steel, 7850 kg/m3): supplier proposes a grade; no heat treatment.",
    "Finish and protection are not in the model: deburr, break sharp edges 0.3 max, oil for transport. Ground faces stay as ground.",
    "Tapped depth is the tap-drill depth of the model; the shop adds tap run-out below it unless the hole is through.",
]
DOWEL_NOTE = "Dowel holes are reamed to H7 at assembly together with the mating support; the shop leaves them at the drilled size of the table."
SHEET_W, SHEET_H = 297.0, 210.0


# ----------------------------------------------------------------------------------------------------------------- data
def fmt(v, digits=2):
    s = f"{v:.{digits}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def load(directory):
    with open(os.path.join(directory, "machined-package.json")) as f:
        return json.load(f)


def iso_tolerance(cls, size):
    for upper, value in ISO_2768[cls]:
        if size <= upper:
            return value
    return ISO_2768[cls][-1][1]


def face_applies(face_name, hole):
    """Which routed face governs a hole: the face is named for holes, bolts, dowels or the bore."""
    name = face_name.lower()
    if hole["kind"] == "bore":
        return "bore" in name
    return any(word in name for word in ("hole", "bolt", "dowel"))


def hole_tolerance(part, hole):
    for face in part["route"]["faces"]:
        if face_applies(face["name"], hole):
            return face
    return None


def face_text(face):
    name = face["name"].lower()
    kind = "position" if any(w in name for w in ("hole", "bolt", "dowel", "bore")) else "flatness"
    ra = "" if face["roughnessRa"] is None else f", Ra {fmt(face['roughnessRa'])}"
    return f"{kind} {fmt(face['tolerance'], 3)}{ra}"


def stock_cut(part, widths):
    """Ground flat bar order: the bar's width is the standard width among the part's two other dimensions."""
    stock = part["route"]["stock"]
    if stock is None:
        return None
    t = stock["thickness"]
    dims = sorted(part["size"])
    match = [d for d in dims if abs(d - t) < 1e-6]
    if not match:
        raise Check(f"{part['id']}: no side equals the stock thickness {t}")
    rest = list(dims)
    rest.remove(match[0])
    if stock.get("kind") == "plate":
        # Ground plate: sawn to the part's two other sides; there is no standard width.
        return {"width": min(rest), "thickness": t, "length": max(rest), "row": stock["id"], "plate": True}
    candidates = [(w, l) for w, l in ((rest[0], rest[1]), (rest[1], rest[0])) if any(abs(w - s) < 1e-6 for s in widths)]
    if not candidates:
        raise Check(f"{part['id']}: neither {rest} is a standard bar width")
    w, l = max(candidates)
    return {"width": w, "thickness": t, "length": l, "row": stock["id"]}


def raw_block(part):
    return [math.ceil((d + 6) / 5) * 5 for d in part["size"]]


def blank_text(part):
    if part["route"]["kind"] == "FoldedSheet":
        f = part["route"]["sheet"]
        return f"sheet {fmt(f['thickness'])} mm; unfold STEP, R{fmt(f['insideRadius'])}"
    if part["route"]["kind"] == "Turned":
        return "round bar blank: supplier proposes diameter and length from the STEP turning axis"
    if part["route"]["kind"] == "CutPlate":
        return "plate blank: supplier proposes thickness and cut allowance from STEP"
    return f"block ≥ {' x '.join(fmt(d) for d in part['rawBlock'])} (indicative)"


def supplier_class(part):
    kind = part["route"]["kind"]
    if kind == "GroundStock":
        return "ground-stock vendor (cut to size) + CNC shop (holes and listed faces)" if part["route"]["faces"] else "ground-stock vendor"
    if kind == "Milled":
        return "CNC milling shop"
    if kind == "Turned":
        return "CNC lathe shop"
    if kind == "FoldedSheet":
        return "sheet metal shop"
    return "plate cutter"


class Check(Exception):
    pass


# --------------------------------------------------------------------------------------------------------------- checks
def check_parts(data, directory, stock_widths):
    errors = []
    limits = data["limits"]
    ids = [p["id"] for p in data["parts"]]
    if len(set(ids)) != len(ids):
        errors.append("duplicate part ids")
    for p in data["parts"]:
        pid = p["id"]
        step = os.path.join(directory, "step", p["step"])
        if not os.path.isfile(step) or os.path.getsize(step) < 1000:
            errors.append(f"{pid}: STEP missing or empty")
        rel = abs(p["stepVolume"] - p["volume"]) / p["volume"]
        p["checks"] = {"stepVolumeDifference": rel}
        if rel > VOLUME_TOLERANCE:
            errors.append(f"{pid}: STEP re-import volume differs from the model by {rel * 100:.3f} % (limit {VOLUME_TOLERANCE * 100:.1f} %)")
        if abs(p["mass"] - p["volume"] * 1e-9 * p["density"]) > 1e-6:
            errors.append(f"{pid}: mass does not follow from volume and density")
        route = p["route"]
        if route["kind"] == "Milled":
            if route["setups"] > limits["maxSetups"]:
                errors.append(f"{pid}: {route['setups']} setups (limit {limits['maxSetups']})")
            if max(p["size"]) > limits["maxExtent"] + 1e-6:
                errors.append(f"{pid}: longest side {max(p['size'])} mm (limit {limits['maxExtent']})")
        if route["kind"] == "GroundStock":
            try:
                p["stockCut"] = stock_cut(p, stock_widths)
            except Check as e:
                errors.append(str(e))
        else:
            p["rawBlock"] = raw_block(p)
        lo, hi = [0, 0, 0], p["size"]
        for h in p["holes"]:
            if not h["present"]:
                errors.append(f"{pid}: hole {h['id']} ({h['kind']} {h['thread'] or ''} at {h['position']}) is not in the solid "
                              f"(wall length {h['wallLength']}, bore empty {h['emptyBore']})")
            for k in range(3):
                if h["position"][k] < lo[k] - 0.01 or h["position"][k] > hi[k] + 0.01:
                    errors.append(f"{pid}: hole {h['id']} mouth {h['position']} lies outside the part {hi}")
        p["checks"]["holesPresent"] = all(h["present"] for h in p["holes"])
        p["checks"]["holeCount"] = len(p["holes"])
        if route["faces"] and not any(True for _ in route["faces"]):
            errors.append(f"{pid}: empty face list")
    if errors:
        raise Check("\n".join(errors))


# ---------------------------------------------------------------------------------------------------- hidden-line views
VIEWS = {
    # name: (u, v, depth) as functions of (x, y, z); depth larger = nearer the viewer. Third angle.
    "TOP": (lambda x, y, z: (x, y, z), 2),
    "FRONT": (lambda x, y, z: (x, z, -y), 1),
    "SIDE": (lambda x, y, z: (y, z, x), 0),
}


def project(name, pts):
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    if name == "TOP":
        return x, y, z
    if name == "FRONT":
        return x, z, -y
    return y, z, x


def rasterise(u, v, w, triangles, res, umin, vmin, shape):
    zbuf = np.full(shape, -np.inf)
    px = (u - umin) * res
    py = (v - vmin) * res
    for tri in triangles:
        xs, ys, ws = px[tri], py[tri], w[tri]
        x0, x1 = int(max(0, math.floor(xs.min()))), int(min(shape[1] - 1, math.ceil(xs.max())))
        y0, y1 = int(max(0, math.floor(ys.min()))), int(min(shape[0] - 1, math.ceil(ys.max())))
        if x1 < x0 or y1 < y0:
            continue
        det = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(det) < 1e-12:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        a = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / det
        b = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / det
        c = 1 - a - b
        inside = (a >= -1e-9) & (b >= -1e-9) & (c >= -1e-9)
        depth = a * ws[0] + b * ws[1] + c * ws[2]
        block = zbuf[y0:y1 + 1, x0:x1 + 1]
        np.maximum(block, np.where(inside, depth, -np.inf), out=block)
    return zbuf


def near_min(zbuf):
    """Minimum of the 3 x 3 neighbourhood: a sample is hidden only when everything around it is nearer."""
    padded = np.pad(zbuf, 1, constant_values=-np.inf)
    stack = [padded[i:i + zbuf.shape[0], j:j + zbuf.shape[1]] for i in range(3) for j in range(3)]
    return np.minimum.reduce(stack)


def on_hole(point, holes):
    """True when a point lies on the wall of one of the holes (a hole edge or seam)."""
    for h in holes:
        d = np.array(h["direction"])
        rel = point - np.array(h["position"])
        t = rel @ d
        radial = np.linalg.norm(rel - t * d)
        if -0.1 <= t <= h["depth"] + 0.1 and abs(radial - h["diameter"] / 2) < 0.06:
            return True
    return False


def view_lines(part):
    """Visible and hidden-hole line segments of each view, in (u, v) mm with the view's minimum corner at (0, 0)."""
    mesh = part["mesh"]
    vertices = np.array(mesh["vertices"]).reshape(-1, 3)
    triangles = np.array(mesh["triangles"]).reshape(-1, 3)
    edges = np.array(mesh["edges"]).reshape(-1, 2, 3)
    size = part["size"]
    res = min(6.0, 2400.0 / max(size))
    result = {}
    holes = part["holes"]
    for name in VIEWS:
        u, v, w = project(name, vertices)
        umin, vmin = 0.0, 0.0
        width, height = float(u.max()), float(v.max())
        shape = (int(math.ceil(height * res)) + 3, int(math.ceil(width * res)) + 3)
        zbuf = rasterise(u, v, w, triangles, res, umin, vmin, shape)
        guard = near_min(zbuf)
        visible, hidden = [], []
        for a, b in edges:
            length = float(np.linalg.norm(b - a))
            n = max(2, int(math.ceil(length * res)) + 1)
            ts = np.linspace(0.0, 1.0, n)
            pts = a[None, :] + ts[:, None] * (b - a)[None, :]
            uu, vv, ww = project(name, pts)
            ix = np.clip((uu * res).astype(int), 0, shape[1] - 1)
            iy = np.clip((vv * res).astype(int), 0, shape[0] - 1)
            hide = guard[iy, ix] > ww + 0.3
            is_hole = on_hole(0.5 * (a + b), holes) if holes else False
            start = None
            for i in range(n + 1):
                state = None if i == n else bool(hide[i])
                if start is None and state is not None:
                    start, mode = i, state
                elif start is not None and state != mode:
                    lo, hi = start, i - 1
                    if hi > lo:
                        seg = ((uu[lo], vv[lo]), (uu[hi], vv[hi]))
                        if not mode:
                            visible.append(seg)
                        elif is_hole:
                            hidden.append(seg)
                    start, mode = (i, state) if state is not None else (None, None)
        result[name] = {"visible": visible, "hidden": hidden}
    return result


# ------------------------------------------------------------------------------------------------------------- drawing
SCALES = [5, 2, 1, 0.5, 0.4, 1 / 3, 0.25, 0.2, 1 / 6, 0.125, 0.1, 0.05]


def scale_name(s):
    return f"{fmt(s)}:1" if s >= 1 else f"1:{fmt(1 / s)}"


class Svg:
    def __init__(self):
        self.out = []

    def line(self, x1, y1, x2, y2, w=0.25, dash=None, color="#000"):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.out.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{w}"{d}/>')

    def path(self, segments, w, dash=None):
        if not segments:
            return
        d = " ".join(f"M{a[0]:.2f} {a[1]:.2f}L{b[0]:.2f} {b[1]:.2f}" for a, b in segments)
        dd = f' stroke-dasharray="{dash}"' if dash else ""
        self.out.append(f'<path d="{d}" fill="none" stroke="#000" stroke-width="{w}" stroke-linecap="round"{dd}/>')

    def text(self, x, y, s, size=2.6, anchor="start", bold=False, color="#000", rotate=None):
        t = f' transform="rotate({rotate} {x:.2f} {y:.2f})"' if rotate else ""
        b = ' font-weight="bold"' if bold else ""
        self.out.append(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}"{b} fill="{color}"{t}>{html.escape(str(s))}</text>')

    def rect(self, x, y, w, h, stroke=0.25, fill="none"):
        self.out.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" stroke="#000" stroke-width="{stroke}"/>')

    def arrow(self, x, y, dx, dy, size=1.6):
        """Arrow head with its tip at (x, y) pointing along (dx, dy)."""
        n = math.hypot(dx, dy)
        dx, dy = dx / n, dy / n
        px, py = -dy, dx
        pts = [(x, y), (x - dx * size * 2.2 + px * size * 0.5, y - dy * size * 2.2 + py * size * 0.5),
               (x - dx * size * 2.2 - px * size * 0.5, y - dy * size * 2.2 - py * size * 0.5)]
        self.out.append('<polygon points="' + " ".join(f"{a:.2f},{b:.2f}" for a, b in pts) + '" fill="#000"/>')

    def dim_h(self, x1, x2, y, label, ext_from, text_size=2.8):
        """Horizontal dimension between x1 and x2 on line y; extension lines from ext_from (a y) to y."""
        for x in (x1, x2):
            self.line(x, ext_from + (1 if y > ext_from else -1), x, y + (1.5 if y > ext_from else -1.5), 0.15)
        self.line(x1, y, x2, y, 0.15)
        self.arrow(x1, y, 1, 0)
        self.arrow(x2, y, -1, 0)
        self.text((x1 + x2) / 2, y - 0.8, label, text_size, "middle")

    def dim_v(self, y1, y2, x, label, ext_from, text_size=2.8):
        for y in (y1, y2):
            self.line(ext_from + (1 if x > ext_from else -1), y, x + (1.5 if x > ext_from else -1.5), y, 0.15)
        self.line(x, y1, x, y2, 0.15)
        self.arrow(x, y1, 0, 1)
        self.arrow(x, y2, 0, -1)
        self.text(x - 0.8, (y1 + y2) / 2, label, text_size, "middle", rotate=-90)

    def render(self):
        return "\n".join(self.out)


def choose_scale(a, b, c, area_w, area_h, gx, gy):
    for s in SCALES:
        if a * s + gx + b * s <= area_w and b * s + gy + c * s <= area_h:
            return s
    return SCALES[-1]


def hole_label(h):
    if h["kind"] == "tapped":
        return f"{h['thread']}" + ("" if h["through"] else f" x {fmt(h['cutDepth'], 1)}")
    if h["kind"] == "dowel":
        return f"Ø{fmt(h['nominal'])} {h['fit']}"
    if h["kind"] == "bore":
        return f"Ø{fmt(h['diameter'])}"
    return f"Ø{fmt(h['diameter'])}"


def hole_row(part, h):
    tol = hole_tolerance(part, h)
    if tol:
        pos = ("Ø" if h["kind"] == "bore" else "±") + fmt(tol["tolerance"], 3)
    else:
        pos = f"ISO 2768-{CLASS_OF_ROUTE[part['route']['toleranceClass']]}"
    if h["kind"] == "tapped":
        kind = f"tapped {h['thread']}"
        dia = f"Ø{fmt(h['diameter'])} drill"
    elif h["kind"] == "dowel":
        kind = f"dowel Ø{fmt(h['nominal'])} {h['fit']}"
        dia = f"Ø{fmt(h['diameter'])} drilled"
    elif h["kind"] == "clearance":
        kind = f"clearance for {h['thread']}"
        dia = f"Ø{fmt(h['diameter'])}"
    else:
        kind = "bore (nut barrel)"
        dia = f"Ø{fmt(h['diameter'])}"
    depth = "through" if h["through"] else f"{fmt(h['cutDepth'], 1)} blind"
    if h["through"] and h["kind"] == "tapped" and h["cutDepth"] < h["depth"] - 0.05:
        depth = f"through ({fmt(h['cutDepth'], 1)})"
    return [h["id"], kind, dia, depth, fmt(h["position"][0], 2), fmt(h["position"][1], 2), fmt(h["position"][2], 2), axis_name(h["direction"]), pos]


def axis_name(d):
    for k, n in enumerate("XYZ"):
        if abs(abs(d[k]) - 1) < 1e-4:
            return ("+" if d[k] > 0 else "-") + n
    return "?"


def notes_for(part):
    route = part["route"]
    cls = CLASS_OF_ROUTE[route["toleranceClass"]]
    size = max(part["size"])
    notes = [f"1. General tolerances ISO 2768-{cls} (linear ±{fmt(iso_tolerance(cls, 30))} to 30 mm, ±{fmt(iso_tolerance(cls, 120))} to 120 mm, "
             f"±{fmt(iso_tolerance(cls, 400))} to 400 mm, ±{fmt(iso_tolerance(cls, 1000))} to 1000 mm). Route class: {route['toleranceClass']}."]
    notes.append("2. Tolerances in the face and hole tables apply as listed; nothing else is toleranced tighter than the general class.")
    if route["kind"] == "GroundStock":
        s = route["stock"]
        notes.append(f"3. Cut from ground flat {s['kind']} {fmt(part['stockCut']['width'])} x {fmt(s['thickness'])}: thickness ±{fmt(s['thicknessTolerance'], 3)}, "
                     f"flatness {fmt(s['flatness'], 3)}, parallelism {fmt(s['parallelism'], 3)} per 100 mm, Ra {fmt(s['roughnessRa'])}. Stock as bought (source: {s.get('source', 'n/a')}{', assumed' if s.get('assumed') else ''}); "
                     "broad faces stay as bought except those marked ground after cutting in the face table; sawn edges as cut.")
    elif route["kind"] == "FoldedSheet":
        f = route["sheet"]
        notes.append(f"3. Cut and form one {fmt(f['thickness'])} mm sheet blank, {f['bends']} bends, inside radius {fmt(f['insideRadius'])} mm. "
                     "Supplier develops the flat pattern from STEP. Corner and tab reliefs are open; no welding. "
                     "Set running edges for at least 2 mm installed clearance; oversize fixing holes allow alignment.")
    elif route["kind"] == "Turned":
        notes.append(f"3. Turn in {route['setups']} setup{'s' if route['setups'] > 1 else ''}; grind the functional faces as listed. "
                     "Choose the round bar blank and turning axis from the STEP geometry. Mill cross-features only where listed on the route.")
    elif route["kind"] == "Milled":
        notes.append(f"3. Milled from the block in {route['setups']} setup{'s' if route['setups'] > 1 else ''} on a 3-axis machine. "
                     f"Indicative raw block {' x '.join(fmt(d) for d in part['rawBlock'])} (size + 3 mm per side).")
    else:
        notes.append("3. Cut the plate profile and holes; supplier proposes the blank from STEP.")
    notes.extend(f"{4 + i}. {t}" for i, t in enumerate(ORDERING))
    if any(h["kind"] == "dowel" for h in part["holes"]):
        notes.append(f"{4 + len(ORDERING)}. {DOWEL_NOTE}")
    notes.append(f"{len(notes) + 1}. Hole positions are of the mouth in the part frame (origin at the minimum corner, axes as the views); see the hole table.")
    return notes


def sheet_svg(part):
    route = part["route"]
    sv = Svg()
    a, b, c = part["size"]
    lines = part["lines"]
    # Views: area, gaps for dimensions.
    ax0, ay0, aw, ah = 17.0, 9.0, 150.0, 138.0
    gx, gy = 22.0, 20.0
    s = choose_scale(a, b, c, aw, ah, gx, gy)
    front_x0 = ax0 + 4
    top_y1 = ay0 + b * s  # bottom edge of top view (sheet y), top view sits above the front view
    front_y0 = top_y1 + gy
    front_y1 = front_y0 + c * s
    side_x0 = front_x0 + a * s + gx
    sv.out.append(f'<g id="views">')

    def place(name, x0, y1):
        """Sheet transform of a view whose minimum corner (0, 0) is at sheet (x0, y1) (y down)."""
        return lambda u, v: (x0 + u * s, y1 - v * s)

    transforms = {"TOP": place("TOP", front_x0, top_y1), "FRONT": place("FRONT", front_x0, front_y1), "SIDE": place("SIDE", side_x0, front_y1)}
    for name, t in transforms.items():
        vis = [(t(*p), t(*q)) for p, q in lines[name]["visible"]]
        hid = [(t(*p), t(*q)) for p, q in lines[name]["hidden"]]
        sv.path(hid, 0.15, "1.2 0.8")
        sv.path(vis, 0.4)
    # Hole centre marks in the view that looks along the hole.
    looks = {"TOP": 2, "FRONT": 1, "SIDE": 0}
    plane = {"TOP": (0, 1), "FRONT": (0, 2), "SIDE": (1, 2)}
    for name, axis in looks.items():
        t = transforms[name]
        for h in part["holes"]:
            if abs(abs(h["direction"][axis]) - 1) > 1e-4:
                continue
            i, j = plane[name]
            x, y = t(h["position"][i], h["position"][j])
            r = h["diameter"] / 2 * s
            k = max(r * 1.4, 1.5)
            sv.line(x - k, y, x + k, y, 0.12, "3 0.6 0.6 0.6")
            sv.line(x, y - k, x, y + k, 0.12, "3 0.6 0.6 0.6")
            sv.text(x + r * 0.8 + 0.6, y - r * 0.8 - 0.4, h["id"], 2.1, color="#06c")
    # Datum corner.
    for name, t in transforms.items():
        x, y = t(0, 0)
        sv.line(x - 2.5, y, x + 2.5, y, 0.12)
        sv.line(x, y - 2.5, x, y + 2.5, 0.12)
    sv.text(*(lambda p: (p[0] - 1, p[1] + 3.5))(transforms["FRONT"](0, 0)), "O", 2.2, "end")
    # View names.
    sv.text(front_x0 + a * s / 2, ay0 - 1.2, "Top (X-Y)", 2.4, "middle", color="#444")
    sv.text(front_x0 + a * s / 2, front_y1 + 15.5, "Front (X-Z)", 2.4, "middle", color="#444")
    sv.text(side_x0 + b * s / 2, front_y1 + 15.5, "Side (Y-Z)", 2.4, "middle", color="#444")
    # Overall dimensions.
    sv.dim_h(front_x0, front_x0 + a * s, front_y1 + 8, fmt(a), front_y1)
    sv.dim_v(ay0, top_y1, front_x0 - 7, fmt(b), front_x0)
    sv.dim_v(front_y0, front_y1, front_x0 - 7, fmt(c), front_x0)
    sv.dim_h(side_x0, side_x0 + b * s, front_y1 + 8, fmt(b), front_y1)
    sv.dim_v(front_y0, front_y1, side_x0 + b * s + 7, fmt(c), side_x0 + b * s)
    sv.out.append("</g>")
    sv.text(ax0 + 1, front_y1 + 15.5, f"Scale {scale_name(s)}", 2.4, color="#444")
    return sv, s


def wrap(text, width):
    lines, line = [], ""
    for word in text.split(" "):
        if line and len(line) + 1 + len(word) > width:
            lines.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    return lines + [line]


def table(sv, x, y, widths, header, rows, size=2.2, row_h=4.1):
    total = sum(widths)
    sv.rect(x, y, total, row_h, 0.2, "#eee")
    cx = x
    for w, t in zip(widths, header):
        sv.text(cx + 0.8, y + row_h - 1.2, t, size, bold=True)
        cx += w
    for i, row in enumerate(rows):
        yy = y + row_h * (i + 1)
        sv.rect(x, yy, total, row_h, 0.12)
        cx = x
        for w, t in zip(widths, row):
            sv.text(cx + 0.8, yy + row_h - 1.2, t, size)
            cx += w
    return y + row_h * (len(rows) + 1)


def sheet_html(part):
    route = part["route"]
    sv, s = sheet_svg(part)
    cls = CLASS_OF_ROUTE[route["toleranceClass"]]
    # Right column: part data, faces, holes.
    x0, w = 184.0, 106.0
    y = 9.0
    data_rows = [
        ("Part", part["partNumber"]),
        ("Description", wrap(part["description"], 64)[0]),
        ("", "".join(wrap(part["description"], 64)[1:2])),
        ("Quantity", f"{part['quantity']} per machine"),
        ("Material", f"{part['material']} (grade: supplier proposes)"),
        ("Process", route["process"]),
        ("Goes to", supplier_class(part)),
    ]
    if route["kind"] == "GroundStock":
        c = part["stockCut"]
        data_rows.append(("Stock", f"{route['stock']['id']}: ground {'plate' if c.get('plate') else 'flat bar'} {fmt(c['width'])} x {fmt(c['thickness'])}, cut to {fmt(c['length'])}"))
    else:
        data_rows.append(("Stock", blank_text(part)))
    data_rows += [
        ("Bends" if route["kind"] == "FoldedSheet" else "Setups", str(route["sheet"]["bends"]) if route["kind"] == "FoldedSheet" else str(route["setups"]) if route["setups"] else "cut only"),
        ("General tol.", f"ISO 2768-{cls}"),
        ("Size / mass", f"{fmt(part['size'][0])} x {fmt(part['size'][1])} x {fmt(part['size'][2])} mm, {fmt(part['mass'], 3)} kg"),
        ("Finish", "deburred, oiled; ground faces as ground"),
    ]
    for k, v in [r for r in data_rows if r != ("", "")]:
        sv.text(x0, y + 3, k, 2.1, color="#444")
        sv.text(x0 + 21, y + 3, v, 2.4 if len(v) < 56 else 2.1)
        sv.line(x0, y + 4.2, x0 + w, y + 4.2, 0.1)
        y += 4.6
    y += 2
    faces = [[f["name"], face_text(f)] for f in route["faces"]] or [["none (stock and cut faces as supplied)", ""]]
    sv.text(x0, y + 2.5, "Machined faces (from the route)", 2.5, bold=True)
    y = table(sv, x0, y + 4, [58, 48], ["Face", "Tolerance (mm)"], faces)
    y += 3
    if part["holes"]:
        sv.text(x0, y + 2.5, "Holes (mouth position in the part frame, mm)", 2.5, bold=True)
        rows = [hole_row(part, h) for h in part["holes"]]
        y = table(sv, x0, y + 4, [7, 20, 14.5, 12.5, 9, 9, 9, 6, 19], ["#", "Type", "Size", "Depth", "X", "Y", "Z", "Dir", "Pos. tol."], rows, size=1.75, row_h=3.7)
    else:
        sv.text(x0, y + 2.5, "Holes: none in the model.", 2.5, bold=True)
    # Notes (bottom left).
    ny = 163.0
    for n in notes_for(part):
        words, line, wrapped = n.split(" "), "", []
        for wd in words:
            if len(line) + len(wd) + 1 > 118:
                wrapped.append(line)
                line = "   " + wd
            else:
                line = (line + " " + wd).strip() if line else wd
        wrapped.append(line)
        for ln in wrapped:
            sv.text(10, ny, ln, 2.0)
            ny += 2.9
    # Frame and title block.
    frame = f"""<rect x="5" y="5" width="287" height="200" fill="none" stroke="#000" stroke-width="0.6"/>
<line x1="5" y1="156" x2="292" y2="156" stroke="#000" stroke-width="0.4"/>
<line x1="185" y1="156" x2="185" y2="205" stroke="#000" stroke-width="0.4"/>"""
    tb = [("Title", part["title"]), ("Drawing no.", f"MEX-HOB-{part['partNumber']}  rev {REV}"),
          ("Material / mass", f"{part['material']}, {fmt(part['mass'], 3)} kg, qty {part['quantity']}"),
          ("Scale / units", f"{scale_name(s)}; mm; third-angle projection"),
          ("Company", COMPANY), ("Date / contact", f"{date.today().isoformat()}  {CONTACT}")]
    rows = ""
    for i, (k, v) in enumerate(tb):
        yy = 162 + 6.8 * i
        rows += (f'<text x="187" y="{yy}" font-size="2.2" fill="#444">{html.escape(k)}</text>'
                 f'<text x="208" y="{yy}" font-size="{2.7 if len(v) < 50 else 2.25}">{html.escape(v)}</text>'
                 f'<line x1="185" y1="{yy + 1.6}" x2="292" y2="{yy + 1.6}" stroke="#000" stroke-width="0.12"/>')
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(part['partNumber'])}</title>
<style>@page {{ size: 297mm 210mm; margin: 0 }} html,body {{ margin:0; padding:0 }} svg {{ display:block; font-family: Arial, Helvetica, sans-serif }}</style></head><body>
<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210">
<rect x="0" y="0" width="297" height="210" fill="#fff"/>
{frame}
{sv.render()}
{rows}
</svg></body></html>""", s


def dxf_for(part):
    a, b, c = part["size"]
    gap = 40.0
    offsets = {"FRONT": (0.0, 0.0), "TOP": (0.0, c + gap), "SIDE": (a + gap, 0.0)}
    out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]

    def line(layer, p, q):
        out.extend(["0", "LINE", "8", layer, "10", f"{p[0]:.4f}", "20", f"{p[1]:.4f}", "30", "0.0", "11", f"{q[0]:.4f}", "21", f"{q[1]:.4f}", "31", "0.0"])

    def text(layer, p, s, h=3.5):
        out.extend(["0", "TEXT", "8", layer, "10", f"{p[0]:.4f}", "20", f"{p[1]:.4f}", "30", "0.0", "40", f"{h:.3f}", "1", s])

    for name, (ox, oy) in offsets.items():
        for p, q in part["lines"][name]["visible"]:
            line(f"VISIBLE_{name}", (p[0] + ox, p[1] + oy), (q[0] + ox, q[1] + oy))
        for p, q in part["lines"][name]["hidden"]:
            line(f"HIDDEN_{name}", (p[0] + ox, p[1] + oy), (q[0] + ox, q[1] + oy))
    looks = {"TOP": (2, (0, 1)), "FRONT": (1, (0, 2)), "SIDE": (0, (1, 2))}
    for name, (axis, (i, j)) in looks.items():
        ox, oy = offsets[name]
        for h in part["holes"]:
            if abs(abs(h["direction"][axis]) - 1) < 1e-4:
                x, y, r = h["position"][i] + ox, h["position"][j] + oy, max(h["diameter"], 3)
                line("CENTRE", (x - r, y), (x + r, y))
                line("CENTRE", (x, y - r), (x, y + r))
                text("TEXT", (x + r * 0.6, y + r * 0.6), h["id"], 2.5)
    line("DIM", (0, -8), (a, -8))
    text("DIM", (a / 2, -14), fmt(a))
    text("TEXT", (0, -30), f"{part['partNumber']} rev {REV} - {part['description']} - {part['material']} - mm, third angle, 1:1")
    text("TEXT", (0, -36), "Hole table and notes: see the PDF sheet. Origin at the minimum corner of each view.")
    return "\n".join(out + ["0", "ENDSEC", "0", "EOF"]) + "\n"


# --------------------------------------------------------------------------------------------------------------- summary
def summary_json(data, directory):
    parts = []
    for p in data["parts"]:
        q = {k: v for k, v in p.items() if k not in ("mesh", "lines")}
        q["supplierClass"] = supplier_class(p)
        q["generalTolerance"] = f"ISO 2768-{CLASS_OF_ROUTE[p['route']['toleranceClass']]}"
        q["drawing"] = f"drawings/{p['id']}.pdf"
        q["holeTable"] = [{"id": h["id"], "kind": h["kind"], "thread": h["thread"], "drillDiameter": h["diameter"], "nominal": h["nominal"],
                           "fit": h["fit"], "depth": h["cutDepth"], "through": h["through"], "position": h["position"], "direction": h["direction"],
                           "positionTolerance": (hole_tolerance(p, h) or {}).get("tolerance")} for h in p["holes"]]
        parts.append(q)
    doc = {"machine": data["machine"], "generated": date.today().isoformat(), "limits": data["limits"], "ordering": ORDERING,
           "totalMass": sum(p["mass"] * p["quantity"] for p in data["parts"]), "parts": parts}
    with open(os.path.join(directory, "parts.json"), "w") as f:
        json.dump(doc, f, indent=1)
    return doc


def rfq(data, directory):
    header = ["Part", "Qty", "Process", "Goes to", "Material / stock", "Finished size (mm)", "Mass (kg)", "Holes", "Tolerances"]
    rows = []
    for p in data["parts"]:
        r = p["route"]
        if r["kind"] == "GroundStock":
            c = p["stockCut"]
            stock = f"{p['material']}; ground {'plate' if c.get('plate') else 'flat bar'} {fmt(c['width'])} x {fmt(c['thickness'])} cut to {fmt(c['length'])}"
        else:
            stock = f"{p['material']}; {blank_text(p)}"
        holes = {}
        for h in p["holes"]:
            key = {"tapped": f"{h['thread']} tapped", "dowel": f"dowel Ø{fmt(h['nominal'])} {h['fit']}", "clearance": f"Ø{fmt(h['diameter'])} clearance",
                   "bore": f"bore Ø{fmt(h['diameter'])}"}[h["kind"]]
            holes[key] = holes.get(key, 0) + 1
        faces = "; ".join(f"{f['name']}: {face_text(f)}" for f in r["faces"]) or "stock and cut faces as supplied"
        tol = f"ISO 2768-{CLASS_OF_ROUTE[r['toleranceClass']]}; {faces}"
        rows.append([p["partNumber"], str(p["quantity"]), r["process"] + (f" ({r['setups']} setups)" if r["kind"] == "Milled" else ""), supplier_class(p), stock,
                     " x ".join(fmt(d) for d in p["size"]), fmt(p["mass"], 3), ", ".join(f"{n} x {k}" for k, n in holes.items()) or "none", tol])
    with open(os.path.join(directory, "parts-rfq.csv"), "w", newline="") as f:
        csv.writer(f).writerows([header] + rows)
    total = sum(p["mass"] * p["quantity"] for p in data["parts"])
    md = ["# Machined parts for the flexspline hobber: request for quotation", "",
          f"Generated {date.today().isoformat()} from the model. One drawing (PDF, DXF) and one STEP per part; quantities are per machine.", "",
          "| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    md += ["| " + " | ".join(c.replace("|", "/") for c in r) + " |" for r in rows]
    md += ["", f"Total mass of the set: {fmt(total, 2)} kg.", "", "## Ordering requirements the model does not hold", ""]
    md += [f"- {t}" for t in ORDERING]
    md += ["- " + DOWEL_NOTE, "- Quote lead time and unit price per part; for the ground-stock parts say whether you drill and tap or only cut and grind.", ""]
    with open(os.path.join(directory, "PARTS_RFQ.md"), "w") as f:
        f.write("\n".join(md))


# ------------------------------------------------------------------------------------------------------------- read back
def read_back(part, html_text, pdf_path, scale):
    problems = []
    for needle in [part["partNumber"], fmt(part["size"][0]), fmt(part["size"][1]), fmt(part["size"][2]), part["material"]] + [h["id"] for h in part["holes"]]:
        if f">{html.escape(needle)}<" not in html_text and html.escape(needle) not in html_text:
            problems.append(f"{part['id']}: sheet lacks {needle!r}")
    for f in part["route"]["faces"]:
        if html.escape(f["name"]) not in html_text:
            problems.append(f"{part['id']}: sheet lacks face {f['name']!r}")
    if shutil.which("pdftotext") and os.path.isfile(pdf_path):
        text = subprocess.run(["pdftotext", "-layout", pdf_path, "-"], capture_output=True, text=True).stdout
        for needle in [part["partNumber"], fmt(part["mass"], 3), scale_name(scale)] + [h["id"] for h in part["holes"]]:
            if needle not in text:
                problems.append(f"{part['id']}: PDF text lacks {needle!r}")
    # Hole table rows against the data: one row per hole, positions to the same digits.
    for h in part["holes"]:
        row = hole_row(part, h)
        for cell in row[4:7]:
            if f">{cell}<" not in html_text:
                problems.append(f"{part['id']}: hole {h['id']} coordinate {cell} not on the sheet")
    return problems


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    directory = sys.argv[1]
    data = load(directory)
    stock_widths = data.get("barWidths") or [10, 12, 15, 16, 20, 25, 30, 32, 40, 50, 60, 80, 100]
    try:
        check_parts(data, directory, stock_widths)
    except Check as e:
        print("CHECK FAILED\n" + str(e), file=sys.stderr)
        sys.exit(1)
    os.makedirs(os.path.join(directory, "drawings"), exist_ok=True)
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser") if shutil.which(c)), None)
    problems = []
    for p in data["parts"]:
        p["lines"] = view_lines(p)
        page, s = sheet_html(p)
        base = os.path.join(directory, "drawings", p["id"])
        with open(base + ".html", "w") as f:
            f.write(page)
        with open(base + ".dxf", "w") as f:
            f.write(dxf_for(p))
        if chrome:
            subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", f"--print-to-pdf={base}.pdf",
                            "file://" + os.path.abspath(base + ".html")], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            problems.append("no Chrome or Chromium on the PATH: PDFs not written")
        problems += read_back(p, page, base + ".pdf", s)
        print(f"{p['id']}: {' x '.join(fmt(d) for d in p['size'])} mm, {fmt(p['mass'], 3)} kg, {len(p['holes'])} holes, scale {scale_name(s)}")
    summary_json(data, directory)
    rfq(data, directory)
    if problems:
        print("CHECK FAILED\n" + "\n".join(problems), file=sys.stderr)
        sys.exit(1)
    print(f"{len(data['parts'])} parts, {fmt(sum(p['mass'] * p['quantity'] for p in data['parts']), 2)} kg; all checks pass")


if __name__ == "__main__":
    main()
