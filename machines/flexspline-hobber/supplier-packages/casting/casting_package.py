#!/usr/bin/env python3
"""Mineral-casting supplier package (base and column) from the model export.

Usage: casting_package.py <export dir> [bed|column ...]   (default: both)

Reads bed-package.json and column-package.json (written by CastingPackageExport.hx next to the STEP files) and
writes, into the same directory:
  casting-summary.json         per casting: outline, volume, mass and the density used, insert list, fixing sockets,
                               cores and cavities, lifting points, tolerance table (source: model or to agree), notes
  <kind>-drawings.dxf          DXF R12, mm, 1:1: the views and sections side by side, one layer per view and kind
  <kind>-drawings.html/.pdf    A3 drawing set: face views with overall dimensions, insert layout and datum faces,
                               sections, schedules, tolerance table and notes

Geometry, masses, positions, tolerances the model holds and the insert list come only from the model. What the model
does not hold (flatness, parallelism, general tolerances, resin system, cure, finish, sealing, lifting hardware) is
printed as "to agree with the supplier"; no number is invented here. The script stops if the drawings disagree with the
JSON, or if the model changed so that a statement in the text no longer holds.
"""
import html
import itertools
import json
import math
import os
import re
import shutil
import subprocess
import sys
from datetime import date

COMPANY = "Materia Expedita, Rua das Oficinas Municipais 15, 2450-060 Nazaré, Portugal"
TOL = 1e-6
AGREE = "to agree with the supplier"
AX = "xyz"
CAST = {
    "bed": dict(drawing="CAS-01", title="Machine base (plinth) mineral casting"),
    "column": dict(drawing="CAS-02", title="Column mineral casting"),
}
# World axis index and sign of the sheet's horizontal and vertical directions for each view direction.
ORIENT = {
    "top": ((0, 1), (1, 1), "looking down -Z"),
    "bottom": ((0, 1), (1, -1), "looking up +Z"),
    "front": ((0, 1), (2, 1), "looking +Y from the y min side"),
    "back": ((0, -1), (2, 1), "looking -Y from the y max side"),
    "left": ((1, -1), (2, 1), "looking +X from the x min side"),
    "right": ((1, 1), (2, 1), "looking -X from the x max side"),
}
FACE_VIEW = {"face-z-max": "top", "face-z-min": "bottom", "face-y-min": "front", "face-y-max": "back",
             "face-x-min": "left", "face-x-max": "right"}
FACE_TITLE = {"face-z-max": "TOP FACE (z max)", "face-z-min": "UNDERSIDE (z min), seen from below",
              "face-y-min": "FRONT FACE (y min)", "face-y-max": "BACK FACE (y max)",
              "face-x-min": "END FACE (x min)", "face-x-max": "END FACE (x max)"}
SECTION_VIEW = {"x": "right", "y": "front", "z": "top"}

# Design statements the text relies on; the script stops if the model stops matching them.
EXPECT = {
    "bed": dict(feet=3, lifting=4, inserts_ground=("tablePlate", "railLeft", "railRight", "driveSeat")),
    "column": dict(feet=0, lifting=4, inserts_ground=("basePlate", "railLeft", "railRight", "driveSeat")),
}

CATEGORIES = [
    (r"^foot\d+-(access|stem)$", "foot pocket (levelling foot access and stem passage)"),
    (r"^work-well$", "work well (table, chip tray)"),
    (r"^chip-drain$", "chip drain"),
    (r"^rotary-union-access$", "rotary union access"),
    (r"^X-drive-channel$", "X-drive channel"),
    (r"^cable-route$", "cable duct"),
    (r"^coolant-front-access$", "coolant tank access opening"),
    (r"^spindle-motor-relief$", "spindle servo relief (running gap from the hob spindle's servo over its whole sweep)"),
    (r"^tray-", "chip-tray ledge and bridge seat"),
    (r"^pocket-", "lightening core (pocket between ribs)"),
    (r"^head-recess-", "column head recess"),
]


def category(cavity_id):
    for pattern, name in CATEGORIES:
        if re.match(pattern, cavity_id):
            return name
    sys.exit(f"cavity '{cavity_id}' has no category in casting_package.py; add it")


def fmt(v, digits=2):
    s = f"{v:.{digits}f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def axis_name(vec):
    for i, c in enumerate(vec):
        if abs(abs(c) - 1) < 1e-6:
            return ("+" if c > 0 else "-") + AX[i].upper()
    return "(" + ", ".join(fmt(c, 4) for c in vec) + ")"


def centre(box):
    return [(a + b) / 2 for a, b in zip(box["min"], box["max"])]


def size(box):
    return [b - a for a, b in zip(box["min"], box["max"])]


def insert_size(ins):
    b = ins["body"]
    if b["kind"] == "box":
        return [b["width"], b["depth"], b["height"]]
    if b["kind"] == "cylinder":
        return [b["diameter"], b["diameter"], b["height"]]
    return [b["diameter"], b["diameter"], b["boreLength"] + b["tailLength"]]


def insert_mark(ins_id):
    return ins_id.replace("trayClampSeat", "S")


# ---------------------------------------------------------------------------------------------------------------
# Model checks
# ---------------------------------------------------------------------------------------------------------------
def check_rows(kind, data, root):
    """Consistency of the exported numbers: mass, outline, insert placement, files."""
    c = data["casting"]
    for got, want in zip(size(c["bounds"]), c["nominal"]):
        if abs(got - want) > 1e-6:
            sys.exit(f"{kind}: casting bounds {size(c['bounds'])} differ from the nominal outline {c['nominal']}")
    mass = c["density"] * c["volumeMm3"] * 1e-9
    if abs(mass - c["massKg"]) > 1e-6 * mass:
        sys.exit(f"{kind}: mass {c['massKg']} kg is not density {c['density']} x volume {c['volumeMm3']} mm3 = {mass}")
    lo, hi = c["bounds"]["min"], c["bounds"]["max"]
    seen = {}
    for ins in data["inserts"]:
        b = ins["bounds"]
        # The exported box is the posed geometry; it must match the pose applied to the local box.
        ax = ins["axes"]
        loc = ins["localBounds"]
        corners = []
        for sx, sy, sz in itertools.product(("min", "max"), repeat=3):
            v = [loc[sx][0], loc[sy][1], loc[sz][2]]
            corners.append([ins["position"][i] + ax["x"][i] * v[0] + ax["y"][i] * v[1] + ax["z"][i] * v[2] for i in range(3)])
        if ins["body"]["kind"] == "box":
            for i in range(3):
                if abs(min(p[i] for p in corners) - b["min"][i]) > 1e-6 or abs(max(p[i] for p in corners) - b["max"][i]) > 1e-6:
                    sys.exit(f"{kind}: insert {ins['id']} pose does not give its exported bounds on {AX[i]}")
        for i in range(3):
            if b["min"][i] < lo[i] - 60 or b["max"][i] > hi[i] + 60:
                sys.exit(f"{kind}: insert {ins['id']} lies far outside the casting")
        f = os.path.join(root, ins["step"])
        if not os.path.exists(f) or os.path.getsize(f) < 1000:
            sys.exit(f"{kind}: missing insert STEP {ins['step']}")
        seen[ins["step"]] = seen.get(ins["step"], 0) + 1
    for s in data["insertSteps"]:
        if seen.get(s["step"]) != s["quantity"]:
            sys.exit(f"{kind}: {s['step']} quantity {s['quantity']} but {seen.get(s['step'])} placements use it")
    if set(seen) != {s["step"] for s in data["insertSteps"]}:
        sys.exit(f"{kind}: insert STEP list does not match the placements")
    for name in c["steps"].values():
        f = os.path.join(root, name)
        if not os.path.exists(f) or os.path.getsize(f) < 10000:
            sys.exit(f"{kind}: missing STEP {name}")
    for cav in data["cavities"] + data["insertRecesses"]:
        for i in range(3):
            if cav["min"][i] < lo[i] - 1e-6 or cav["max"][i] > hi[i] + 1e-6:
                sys.exit(f"{kind}: cavity {cav['id']} lies outside the casting")
    ids = [i["id"] for i in data["inserts"]]
    if sorted(ids) != sorted(r["owner"] for r in data["insertRecesses"]):
        sys.exit(f"{kind}: every insert needs exactly one recess in the casting")


def check_layout(kind, data):
    """The text below assumes these design facts; fail if the model moved them."""
    want = EXPECT[kind]
    c = data["casting"]
    if c["datumScheme"] is not None:
        sys.exit(f"{kind}: the model now declares a datum scheme; the tolerance table says it has none. Update the script")
    if len(c["levellingFootPlates"]) != want["feet"]:
        sys.exit(f"{kind}: {len(c['levellingFootPlates'])} levelling plates, text assumes {want['feet']}")
    if len(data["liftingPoints"]) != want["lifting"]:
        sys.exit(f"{kind}: {len(data['liftingPoints'])} lifting points, text assumes {want['lifting']}")
    by = {i["id"]: i for i in data["inserts"]}
    for i in want["inserts_ground"]:
        if i not in by or by[i]["finish"] != "ground-after-casting" or by[i]["datumFace"] is None:
            sys.exit(f"{kind}: insert {i} is no longer a ground-after-casting datum seat")
    for i in data["inserts"]:
        if i["precisionInterfaces"]:
            sys.exit(f"{kind}: insert {i['id']} declares precision interfaces; the tolerance table assumes none")
    if kind == "bed" and sum(1 for x in data["cavities"] if x["id"].startswith("foot")) != 6:
        sys.exit("bed: expected 3 foot pockets of two cavities each")
    for cav in data["cavities"]:
        category(cav["id"])
    names = {s["id"] for s in data["sections"]}
    for need in FACE_VIEW:
        if need not in names:
            sys.exit(f"{kind}: missing face section {need}")


# ---------------------------------------------------------------------------------------------------------------
# What the model gives, and what is open
# ---------------------------------------------------------------------------------------------------------------
def sockets(data):
    return [i for i in data["inserts"] if i["role"] == "threaded-anchor"]


def steel_inserts(data):
    return [i for i in data["inserts"] if i["role"] != "threaded-anchor"]


def proud(data):
    """How far each insert stands beyond the casting envelope, by face."""
    lo, hi = data["casting"]["bounds"]["min"], data["casting"]["bounds"]["max"]
    rows = []
    for i in data["inserts"]:
        b = i["bounds"]
        for k in range(3):
            if b["max"][k] > hi[k] + TOL:
                rows.append((i["id"], "+" + AX[k].upper(), b["max"][k] - hi[k]))
            if b["min"][k] < lo[k] - TOL:
                rows.append((i["id"], "-" + AX[k].upper(), lo[k] - b["min"][k]))
    return rows


def tolerance_rows(kind, data):
    rows = []
    ground = [i for i in data["inserts"] if i["datumFace"] is not None]
    for tol in sorted({i["positionToleranceMm"] for i in ground}):
        ids = ", ".join(i["id"] for i in ground if i["positionToleranceMm"] == tol)
        rows.append(("Finished insert mounting plane (position and face parallelism together)",
                     f"{fmt(tol, 4)} mm of nominal: position error plus tilt x longest face dimension. Inserts: {ids}",
                     "model: insert positionTolerance, checked by FoundationIntegrity"))
    for tol in sorted({i["positionToleranceMm"] for i in sockets(data)}):
        ids = ", ".join(insert_mark(i["id"]) for i in sockets(data) if i["positionToleranceMm"] == tol)
        rows.append(("Fixing socket position, as cast", f"{fmt(tol, 3)} mm. Sockets: {ids}", "model: socket positionTolerance"))
    if [i for i in data["inserts"] if i["role"] == "lifting-mount"]:
        lift = [i for i in data["inserts"] if i["role"] == "lifting-mount"]
        rows.append(("Lifting plates (as supplied, not ground)",
                     "Model carries the same 0.01 mm position budget; plates are as supplied (finish: as-supplied). "
                     "Lifting is by placeholder M12 threads, not qualified: " + ", ".join(i["id"] for i in lift),
                     "model: positionTolerance (placeholder)"))
    rows.append(("Insert finishing route",
                 "Ground steel mounting faces after cure (finish: ground-after-casting). Never machine datum faces into the "
                 "mineral cast. Grinding stock and replicated layer thickness are not modelled (no precision interfaces "
                 "declared): the steel is cast at final thickness", "model + MACHINE_DESIGN"))
    top = sorted({(i, f, round(d, 3)) for i, f, d in proud(data) if i not in ("liftLeft", "liftRight")})
    if top:
        rows.append(("Mounting faces stand proud of the cast surface",
                     "; ".join(f"{i} {f} {fmt(d)} mm" for i, f, d in top) + " (nominal, from the model)",
                     "model geometry; tolerance on the proud height: " + AGREE))
    if kind == "bed":
        rows.append(("Levelling foot pocket roofs (3)",
                     "Full-area contact needs a replicated or bedded bearing face on the pocket roof, using a removable "
                     "released master; no machining of the mineral cast (MACHINE_DESIGN). Flatness and parallelism to the "
                     "rail seats: " + AGREE, "MACHINE_DESIGN; numbers " + AGREE))
    rows += [
        ("Insert location as cast, before finishing", AGREE + " (the model gives the finished 0.01 mm budget only)", AGREE),
        ("Flatness of each finished insert plane", AGREE, AGREE),
        ("Parallelism of the two guide seat planes to each other",
         AGREE + " (assembly step in MACHINE_DESIGN: indicate the second guide parallel to the master)", AGREE),
        ("Parallelism / perpendicularity of the guide seat planes to the sole", AGREE, AGREE),
        ("Sole (underside) flatness", AGREE, AGREE),
        ("Ra of ground steel faces; Ra of the cast mineral faces", AGREE, AGREE),
        ("Overall outline tolerance and shrinkage allowance", AGREE, AGREE),
        ("Position of cores and cavities", AGREE, AGREE),
        ("Thread class of tapped inserts",
         "as modelled: " + ", ".join(sorted({f"{h['thread']} {h['threadClass']}" for i in data["inserts"] for h in i["holes"] if h["thread"]})), "model (class as listed in the holes table)"),
        ("Mass of the finished casting", "model: " + fmt(data["casting"]["massKg"], 1) + " kg at " + fmt(data["casting"]["density"], 0)
         + " kg/m3 (assumed density); tolerance " + AGREE, "model value; tolerance " + AGREE),
    ]
    return rows


def notes(kind, data):
    c = data["casting"]
    n = [
        "Material: epoxy-bound mineral casting (epoxy granite). The model carries only assumed bulk values: density "
        f"{fmt(c['density'], 0)} kg/m3, Young's modulus {fmt(c['youngsModulusPa'] / 1e9, 0)} GPa, damping loss factor "
        f"{fmt(c['dampingLossFactor'], 3)}, thermal expansion {fmt(c['thermalExpansionPerK'] * 1e6, 0)} um/m/K, thermal "
        f"conductivity {fmt(c['thermalConductivityWPerMK'], 0)} W/m/K. These are not test results: state your resin "
        "system, filler grading and the datasheet values for your mix.",
        "Steel inserts are located in the mould as separate parts and cast in (STEP: the casting with inserts). Insert steel is "
        "C45 (EN 1.0503), minimum tensile strength 580 MPa as modelled, except the PFEIFER fixing sockets (1.4571). "
        "Insert supply: we make or buy the inserts and fixing sockets and supply them free of charge to you. As an "
        "option, price making the C45 inserts yourself from the insert STEP files (the sockets stay bought parts).",
        "Threaded holes marked 'before casting' in the schedule are in the steel at the pour: protect the threads and the "
        "entry faces from resin. Holes marked 'after casting' are made by us later and are not in the insert STEP files.",
        "Cure and stress relief: the model holds no cure schedule. Please state your cure and ageing schedule and the "
        "earliest time after casting at which the steel faces may be ground or replicated; the castings should be stress "
        "relieved or aged as your process requires before the steel faces are finished.",
        "The steel mounting faces are ground (or replicated) after cure; the datum faces are the sole and these steel "
        "faces. The mineral surface is not machined. Which faces are datums, and their lettering, is to agree: the model "
        "has no datum scheme.",
        "Surface finish of mineral faces, sealing, and paint or lining: " + AGREE + ". The model has none. Faces that "
        "see coolant, the work well, chip drain and front chamber, need a sealed or lined surface (the tank lining and "
        "plumbing are design provisions, not designed).",
        "Do not add machined faces, inserts or draft to the geometry without agreeing it with us. Mould draft, parting "
        "line, pouring and vent positions and any shrinkage allowance are yours to propose.",
        "Dimensions are mm. The STEP files are authoritative for shape. The coordinate frame is the casting frame: x and y "
        "are centred on the outline, z = 0 on the cast underside, +z up.",
    ]
    if kind == "bed":
        n.append("Levelling feet: three removable 110 x 110 x 15 mm ground steel plates sit under the pocket roofs; they "
                 "are not cast in and not in the casting STEP. The roof bearing faces are replicated or bedded, never machined.")
        n.append("Coolant: the casting reserves 7.92 L for a separate coolant tank in the front chamber below and forward of the "
                 "guide seats, with a drain and a front service opening; the tank is a separate lined vessel.")
    return n


GAPS = [
    "Lifting hardware is not designed. The model has four placeholder M12 threads in steel lifting plates; "
    "MACHINE_DESIGN says they are not qualified lifting points. Lifting method, load rating and the mass the foundry "
    "will handle are open.",
    "No datum scheme and no flatness, parallelism, perpendicularity or general-tolerance numbers in the model; all are "
    "'to agree'. The only numeric tolerances in the model are the insert position budgets (0.01 mm steel seats, 0.5 mm sockets).",
    "No grinding stock or replication layer is modelled (no precision interfaces declared); the inserts are at final thickness.",
    "No mould draft, parting line, casting orientation, pouring or vent position, wall-thickness rule or shrinkage allowance.",
    "Cure schedule, resin system, filler grading, surface finish, sealing, paint and coolant lining are not in the model.",
    "The M6/M8 flange bolts that anchor each insert (see the anchors column) are modelled as full hex flange bolts in the "
    "insert; how they are locked (shoulder or retaining compound) and who fits them is to agree.",
    "Corrosion protection of the C45 inserts before and after casting is not specified.",
    "Quantities, delivery and the inspection report you provide (CMM of insert faces) are in the covering email.",
]


# ---------------------------------------------------------------------------------------------------------------
# Summary JSON
# ---------------------------------------------------------------------------------------------------------------
def lifting_rows(data):
    by = {i["id"]: i for i in data["inserts"]}
    out = []
    for p in data["liftingPoints"]:
        ins = by[p["insertId"]]
        hole = next(h for h in ins["holes"] if h["id"] == p["holeId"])
        out.append({
            "id": p["id"], "insert": p["insertId"], "hole": p["holeId"], "thread": hole["thread"],
            "depth": hole["depth"], "entry": hole["entry"], "direction": hole["direction"],
            "status": "placeholder only: MACHINE_DESIGN says lifting hardware is deferred and these are not "
                      "qualified lifting points",
        })
    return sorted(out, key=lambda r: r["id"])


def summary(kind, data):
    c = data["casting"]
    lo, hi = c["bounds"]["min"], c["bounds"]["max"]
    ins_all = data["inserts"]
    envelope_lo = [min([lo[k]] + [i["bounds"]["min"][k] for i in ins_all]) for k in range(3)]
    envelope_hi = [max([hi[k]] + [i["bounds"]["max"][k] for i in ins_all]) for k in range(3)]
    steel = sum(i["massKg"] for i in ins_all) + sum(a["massKg"] for i in ins_all for a in i["anchors"])
    cats = {}
    for cav in data["cavities"]:
        d = cats.setdefault(category(cav["id"]), {"count": 0, "volumeL": 0.0})
        d["count"] += 1
        d["volumeL"] += cav["volumeMm3"] / 1e6
    return {
        "name": CAST[kind]["title"],
        "drawing": CAST[kind]["drawing"],
        "files": {
            "casting": c["steps"]["body"],
            "castingWithInserts": c["steps"]["withInserts"],
            "castingWithInsertsLight": c["steps"]["withInsertsLight"],
            "inserts": [{"step": s["step"], "quantity": s["quantity"], "firstPlacement": s["firstPlacement"]}
                        for s in data["insertSteps"]],
            "drawings": f"{kind}-drawings.pdf", "dxf": f"{kind}-drawings.dxf",
        },
        "frame": data["units"],
        "outline": {
            "castingMm": size(c["bounds"]), "min": lo, "max": hi,
            "withProudSteelMm": [b - a for a, b in zip(envelope_lo, envelope_hi)],
            "withProudSteelMin": envelope_lo, "withProudSteelMax": envelope_hi,
        },
        "volumeMm3": c["volumeMm3"], "volumeLitres": c["volumeMm3"] / 1e6,
        "massKg": c["massKg"],
        "massBasis": f"as-cast volume x {fmt(c['density'], 0)} kg/m3 (assumed density of the model's 'mineral cast' "
                     "material); steel inserts and anchors are extra",
        "density": c["density"], "densityAssumed": c["propertiesAssumed"],
        "centreOfMass": c["centreOfMass"],
        "steelMassKg": steel, "castAssemblyMassKg": c["massKg"] + steel,
        "materialProperties": {k: c[k] for k in ("youngsModulusPa", "dampingLossFactor", "thermalExpansionPerK",
                                                  "thermalConductivityWPerMK", "propertiesAssumed")},
        "supply": {
            "inserts": "supplied free of charge by us (made or bought by us); option: you make the C45 inserts from the "
                       "insert STEP files and price it separately",
            "fixingSockets": "bought by us (PFEIFER 05.250.064.040 or equivalent) and supplied to you; not made by you",
        },
        "inserts": [{
            "id": i["id"], "mark": insert_mark(i["id"]), "role": i["role"], "label": i["label"], "step": i["step"],
            "size": insert_size(i), "body": i["body"], "material": i["material"], "stockMaterial": i["stockMaterial"],
            "origin": i["origin"], "finish": i["finish"], "massKg": i["massKg"],
            "position": i["position"], "orientation": {"axes": i["axes"], "mountNormal": i["mountNormal"],
                                                       "mountNormalName": axis_name(i["mountNormal"])},
            "bounds": i["bounds"], "centre": centre(i["bounds"]),
            "positionToleranceMm": i["positionToleranceMm"],
            "datumFace": i["datumFace"],
            "holes": i["holes"],
            "anchors": [{"id": a["id"], "description": a["description"], "position": a["position"]} for a in i["anchors"]],
        } for i in ins_all],
        "fixingSockets": [{
            "id": s["id"], "mark": insert_mark(s["id"]), "thread": s["holes"][0]["thread"],
            "threadClass": s["holes"][0]["threadClass"], "boreDepth": s["holes"][0]["depth"],
            "fullThreadDepth": s["holes"][0]["threadDepth"], "position": s["position"],
            "entry": s["holes"][0]["entry"], "direction": s["holes"][0]["direction"],
            "length": s["body"]["boreLength"] + s["body"]["tailLength"],
            "positionToleranceMm": s["positionToleranceMm"], "origin": s["origin"],
            "step": s["step"],
        } for s in sockets(data)],
        "datumFaces": [{
            "id": "sole",
            "description": "cast underside, z = 0" if kind == "bed" else "ground sole face of the basePlate insert",
            "source": "supplier-package convention (sole and precision faces are the datums); the model has no "
                      "datum scheme; lettering " + AGREE,
        }] + [i["datumFace"] | {"description": f"mounting face of {i['id']}, " + i["finish"].replace("-", " ")}
              for i in ins_all if i["datumFace"] is not None],
        "cavities": [{
            "id": cav["id"], "category": category(cav["id"]), "kind": cav["kind"], "openings": cav["openings"],
            "volumeMm3": cav["volumeMm3"], "min": cav["min"], "max": cav["max"],
            "draftDegrees": cav["draftDegrees"], "cornerRadius": cav["cornerRadius"],
        } for cav in data["cavities"]],
        "cavitiesByCategory": cats,
        "insertRecesses": [{"id": r["id"], "insert": r["owner"], "volumeMm3": r["volumeMm3"]} for r in data["insertRecesses"]],
        "coolantTank": {
            "reservedLitres": c["coolantReservedLitres"],
            "note": "stated in MACHINE_DESIGN and FlexsplineHobberBaseReport as a constant for a separate tank in the "
                    "front chamber below and forward of the guide seats, with the chip drain and the front service "
                    "opening; it is not derived from a single cavity, so it is not checked against the cavity volumes",
        } if kind == "bed" else None,
        "footPockets": [{"x": f["x"], "y": f["y"], "plate": f["plate"]} for f in c["levellingFootPlates"]],
        "liftingPoints": lifting_rows(data),
        "liftingNote": "Lifting hardware is deferred in MACHINE_DESIGN; the points above are placeholders. "
                       "Lifting method is to agree.",
        "tolerances": [{"item": a, "requirement": b, "source": s} for a, b, s in tolerance_rows(kind, data)],
        "notes": notes(kind, data),
        "openQuestions": GAPS,
    }


# ---------------------------------------------------------------------------------------------------------------
# Drawing scene
# ---------------------------------------------------------------------------------------------------------------
class View:
    def __init__(self, vid, title, orient, extent):
        self.id, self.title, self.orient = vid, title, orient
        self.h, self.v, self.look = ORIENT[orient]
        self.items = []
        self.extent = extent  # (h0, h1, v0, v1) in sheet coordinates

    def p(self, pt):
        return (self.h[1] * pt[self.h[0]], self.v[1] * pt[self.v[0]])

    def box(self, b):
        a, c = self.p(b["min"]), self.p(b["max"])
        return (min(a[0], c[0]), min(a[1], c[1]), max(a[0], c[0]), max(a[1], c[1]))

    def poly(self, layer, pts, closed=True, **attrs):
        self.items.append(dict(t="poly", layer=layer, pts=pts, closed=closed, attrs=attrs))

    def rect(self, layer, b, **attrs):
        x0, y0, x1, y1 = b
        self.poly(layer, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], True, bounds=b, **attrs)

    def text(self, at, s, h, anchor="start", layer="TEXT"):
        self.items.append(dict(t="text", layer=layer, at=at, s=s, h=h, anchor=anchor))

    def dim(self, a, b, off, s, value, name):
        self.items.append(dict(t="dim", layer="DIMS", a=a, b=b, off=off, s=s, value=value, name=name))


def make_extent(boxes, margin):
    x0 = min(b[0] for b in boxes) - margin
    y0 = min(b[1] for b in boxes) - margin
    x1 = max(b[2] for b in boxes) + margin
    y1 = max(b[3] for b in boxes) + margin
    return (x0, x1, y0, y1)


def all_boxes(view, data):
    boxes = [view.box(data["casting"]["bounds"])]
    boxes += [view.box(i["bounds"]) for i in data["inserts"]]
    return boxes


def outline(view, data, dims=True):
    """Casting outline (layer OUTLINE) and, optionally, its overall dimensions."""
    b = view.box(data["casting"]["bounds"])
    view.rect("OUTLINE", b, outline="casting")
    if dims:
        size_h, size_v = b[2] - b[0], b[3] - b[1]
        m = max(size_h, size_v)
        view.dim((b[0], b[1]), (b[2], b[1]), -0.07 * m, fmt(size_h, 2), size_h, "h")
        view.dim((b[2], b[1]), (b[2], b[3]), 0.07 * m, fmt(size_v, 2), size_v, "v")


def section_view(data, sec, vid, title, orient, dims, inserts_cut=False):
    c = data["casting"]
    base = View(vid, title, orient, (0, 1, 0, 1))
    ob = base.box(c["bounds"])
    boxes = [ob] + [base.box(i["bounds"]) for i in data["inserts"]] if dims else [ob]
    m = 0.15 * max(ob[2] - ob[0], ob[3] - ob[1])
    base.extent = make_extent(boxes, m)
    outline(base, data, dims=dims)
    for k, cont in enumerate(sec["contours"]):
        base.poly("SECTION", [base.p(p) for p in cont["points"]], True, depth=cont["depth"])
    if inserts_cut:
        axis = "xyz".index(sec["axis"])
        for i in data["inserts"]:
            b = i["bounds"]
            if b["min"][axis] - TOL <= sec["offset"] <= b["max"][axis] + TOL:
                bb = dict(min=list(b["min"]), max=list(b["max"]))
                bb["min"][axis] = bb["max"][axis] = sec["offset"]
                base.rect("STEEL_CUT", base.box(bb), steel=i["id"])
    return base


def face_view(data, sec):
    name = sec["id"]
    v = section_view(data, sec, name.upper(), FACE_TITLE[name] + f" - section {fmt(sec['offset'], 1)} mm inside the face",
                     FACE_VIEW[name], True)
    return v


def cut_view(data, sec):
    axis = sec["axis"]
    v = section_view(data, sec, sec["id"].upper(), f"SECTION {axis.upper()} = {fmt(sec['offset'], 1)}  ({ORIENT[SECTION_VIEW[axis]][2]})",
                     SECTION_VIEW[axis], False, inserts_cut=True)
    return v


def layout_view(data, orient, name):
    v = View("LAYOUT-" + orient.upper(), name, orient, (0, 1, 0, 1))
    ob = v.box(data["casting"]["bounds"])
    boxes = all_boxes(v, data)
    v.extent = make_extent(boxes, 0.15 * max(ob[2] - ob[0], ob[3] - ob[1]))
    outline(v, data, dims=True)
    text_h = 0.012 * max(ob[2] - ob[0], ob[3] - ob[1])
    for i in data["inserts"]:
        b = v.box(i["bounds"])
        v.rect("INSERTS", b, insert=i["id"], role=i["role"])
        mark = insert_mark(i["id"])
        if orient == "top":
            v.text(((b[0] + b[2]) / 2, (b[1] + b[3]) / 2 + 0.5 * text_h), mark, text_h, "middle")
        d = i["datumFace"]
        if d is not None:
            n = d["normal"]
            axis = next(k for k in range(3) if abs(n[k]) > 0.5)
            ib = i["bounds"]
            plane = ib["max"][axis] if n[axis] > 0 else ib["min"][axis]
            if axis in (v.h[0], v.v[0]):
                lo_p = {"min": list(ib["min"]), "max": list(ib["max"])}
                lo_p["min"][axis] = lo_p["max"][axis] = plane
                db = v.box(lo_p)
                v.poly("DATUM", [(db[0], db[1]), (db[2], db[3])], False, datum=d["id"])
            else:
                v.rect("DATUM", b, datum=d["id"])
    return v


def build_views(kind, data):
    secs = {s["id"]: s for s in data["sections"]}
    views = {n: face_view(data, secs[n]) for n in FACE_VIEW}
    layout = {o: layout_view(data, o, t) for o, t in (("top", "INSERT LAYOUT, plan (steel and fixing sockets; red = precision datum faces)"),
                                                      ("front", "INSERT LAYOUT, front elevation"),
                                                      ("right", "INSERT LAYOUT, end elevation from x max"))}
    cuts = [cut_view(data, s) for s in data["sections"] if s["id"].startswith("cut-")]
    return views, layout, cuts


# ---------------------------------------------------------------------------------------------------------------
# DXF
# ---------------------------------------------------------------------------------------------------------------
def view_grid(views, per_row=4, gap=200):
    out, x, y, row_h, n = {}, 0.0, 0.0, 0.0, 0
    for v in views:
        w, h = v.extent[1] - v.extent[0], v.extent[3] - v.extent[2]
        if n and n % per_row == 0:
            x, y, row_h = 0.0, y - row_h - gap, 0.0
        out[v.id] = (x - v.extent[0], y - v.extent[3])
        x += w + gap
        row_h = max(row_h, h)
        n += 1
    return out


def write_dxf(views, path):
    place = view_grid(views)
    out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]

    def line(layer, a, b):
        out.extend(["0", "LINE", "8", layer, "10", f"{a[0]:.4f}", "20", f"{a[1]:.4f}", "30", "0.0",
                    "11", f"{b[0]:.4f}", "21", f"{b[1]:.4f}", "31", "0.0"])

    def text(layer, at, s, h):
        out.extend(["0", "TEXT", "8", layer, "10", f"{at[0]:.4f}", "20", f"{at[1]:.4f}", "30", "0.0", "40", f"{h:.3f}", "1", s])

    for v in views:
        ox, oy = place[v.id]
        th = 0.012 * max(v.extent[1] - v.extent[0], v.extent[3] - v.extent[2])
        text(f"{v.id}_TEXT", (v.extent[0] + ox, v.extent[3] + oy + th), v.title.replace("→", "->"), th * 1.4)
        for it in v.items:
            layer = f"{v.id}_{it['layer']}"
            if it["t"] == "poly":
                pts = [(p[0] + ox, p[1] + oy) for p in it["pts"]]
                out.extend(["0", "POLYLINE", "8", layer, "66", "1", "70", "1" if it["closed"] else "0"])
                for p in pts:
                    out.extend(["0", "VERTEX", "8", layer, "10", f"{p[0]:.4f}", "20", f"{p[1]:.4f}", "30", "0.0"])
                out.extend(["0", "SEQEND", "8", layer])
            elif it["t"] == "text":
                text(layer, (it["at"][0] + ox, it["at"][1] + oy), it["s"], it["h"])
            elif it["t"] == "dim":
                a, b, off = it["a"], it["b"], it["off"]
                if it["name"] == "h":
                    a2, b2 = (a[0] + ox, a[1] + off + oy), (b[0] + ox, b[1] + off + oy)
                    line(layer, (a[0] + ox, a[1] + oy), a2)
                    line(layer, (b[0] + ox, b[1] + oy), b2)
                    line(layer, a2, b2)
                    text(layer, ((a2[0] + b2[0]) / 2, a2[1] - th * 1.5), it["s"], th)
                else:
                    a2, b2 = (a[0] + off + ox, a[1] + oy), (b[0] + off + ox, b[1] + oy)
                    line(layer, (a[0] + ox, a[1] + oy), a2)
                    line(layer, (b[0] + ox, b[1] + oy), b2)
                    line(layer, a2, b2)
                    text(layer, (a2[0] + th * 0.5, (a2[1] + b2[1]) / 2), it["s"], th)
    out += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
    return place


def read_dxf(path):
    """Layers -> list of closed/open polylines, as read back from the file."""
    rows = [l.rstrip("\n") for l in open(path)]
    pairs = list(zip(rows[0::2], rows[1::2]))
    polys, cur, i = {}, None, 0
    while i < len(pairs):
        code, val = pairs[i]
        if code == "0" and val == "POLYLINE":
            layer = next(v for c, v in pairs[i + 1:i + 6] if c == "8")
            cur = (layer, [])
        elif code == "0" and val == "VERTEX" and cur is not None:
            x = float(next(v for c, v in pairs[i + 1:i + 8] if c == "10"))
            y = float(next(v for c, v in pairs[i + 1:i + 8] if c == "20"))
            cur[1].append((x, y))
        elif code == "0" and val == "SEQEND" and cur is not None:
            polys.setdefault(cur[0], []).append(cur[1])
            cur = None
        i += 1
    return polys


# ---------------------------------------------------------------------------------------------------------------
# SVG / HTML
# ---------------------------------------------------------------------------------------------------------------
COLOURS = dict(SECTION="#d4dae0", STEEL_CUT="#8fb7e0", INSERTS="#a9c9ea", DATUM="#d01010")


def svg(view, w_mm, h_mm, with_title=True):
    e0, e1, f0, f1 = view.extent
    w, h = e1 - e0, f1 - f0
    scale = min(w_mm / w, h_mm / h)
    th = max(w, h) * 0.022

    def X(x):
        return f"{x - e0:.2f}"

    def Y(y):
        return f"{f1 - y:.2f}"

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w * scale:.1f}mm" height="{h * scale:.1f}mm" '
           f'viewBox="0 0 {w:.2f} {h:.2f}" data-view="{view.id}" data-scale="{scale:.5f}">']
    sections = [it for it in view.items if it["t"] == "poly" and it["layer"] == "SECTION"]
    if sections:
        d = " ".join("M" + " L".join(f"{X(p[0])},{Y(p[1])}" for p in it["pts"]) + " Z" for it in sections)
        out.append(f'<path d="{d}" fill="{COLOURS["SECTION"]}" fill-rule="evenodd" stroke="#222" stroke-width="0.9" '
                   f'vector-effect="non-scaling-stroke"/>')
    for it in view.items:
        if it["t"] == "poly" and it["layer"] != "SECTION":
            pts = " ".join(f"{X(p[0])},{Y(p[1])}" for p in it["pts"])
            a = it["attrs"]
            attrs = f'data-view="{view.id}" data-layer="{it["layer"]}"'
            if "insert" in a:
                attrs += f' data-insert="{a["insert"]}"'
            if "bounds" in a:
                attrs += ' data-bounds="' + ",".join(f"{x:.4f}" for x in a["bounds"]) + '"'
            if a.get("outline"):
                attrs += ' data-outline="casting"'
            if it["layer"] == "OUTLINE":
                out.append(f'<polygon points="{pts}" fill="none" stroke="#000" stroke-width="1.4" '
                           f'vector-effect="non-scaling-stroke" {attrs}/>')
            elif it["layer"] == "STEEL_CUT":
                out.append(f'<polygon points="{pts}" fill="{COLOURS["STEEL_CUT"]}" stroke="#1d4f8a" stroke-width="0.8" '
                           f'vector-effect="non-scaling-stroke" {attrs}/>')
            elif it["layer"] == "INSERTS":
                out.append(f'<polygon points="{pts}" fill="{COLOURS["INSERTS"]}" fill-opacity="0.55" stroke="#1d4f8a" '
                           f'stroke-width="0.8" vector-effect="non-scaling-stroke" {attrs}/>')
            elif it["layer"] == "DATUM":
                tag = "polyline" if not it["closed"] else "polygon"
                out.append(f'<{tag} points="{pts}" fill="none" stroke="{COLOURS["DATUM"]}" stroke-width="3" '
                           f'vector-effect="non-scaling-stroke" data-datum="{a["datum"]}" {attrs}/>')
        elif it["t"] == "text":
            out.append(f'<text x="{X(it["at"][0])}" y="{Y(it["at"][1])}" font-size="{it["h"]:.2f}" text-anchor="{it["anchor"]}" '
                       f'fill="#111">{html.escape(it["s"])}</text>')
        elif it["t"] == "dim":
            a, b, off = it["a"], it["b"], it["off"]
            if it["name"] == "h":
                a2, b2 = (a[0], a[1] + off), (b[0], b[1] + off)
                tx, ty, anchor, rot = (a2[0] + b2[0]) / 2, a2[1] - th * 1.1, "middle", ""
            else:
                a2, b2 = (a[0] + off, a[1]), (b[0] + off, b[1])
                tx, ty = a2[0] + th * 1.2, (a2[1] + b2[1]) / 2
                anchor, rot = "middle", f' transform="rotate(90 {X(tx)} {Y(ty)})"'
            out.append(f'<g stroke="#006" stroke-width="0.8" vector-effect="non-scaling-stroke" data-dim="{it["name"]}" '
                       f'data-value="{it["value"]:.4f}" data-view="{view.id}">'
                       f'<line x1="{X(a[0])}" y1="{Y(a[1])}" x2="{X(a2[0])}" y2="{Y(a2[1])}"/>'
                       f'<line x1="{X(b[0])}" y1="{Y(b[1])}" x2="{X(b2[0])}" y2="{Y(b2[1])}"/>'
                       f'<line x1="{X(a2[0])}" y1="{Y(a2[1])}" x2="{X(b2[0])}" y2="{Y(b2[1])}"/></g>'
                       f'<text x="{X(tx)}" y="{Y(ty)}" font-size="{th * 1.1:.2f}" text-anchor="{anchor}" fill="#006"{rot}>'
                       f'{html.escape(it["s"])}</text>')
    out.append("</svg>")
    return "".join(out), scale


def cell(view, w_mm, h_mm):
    s, scale = svg(view, w_mm, h_mm - 7)
    return (f'<div class="cell" style="width:{w_mm}mm;height:{h_mm}mm"><div class="vt">{html.escape(view.title)} '
            f'<span>scale about 1:{1 / scale:.1f} on A3; {view.look}</span></div>{s}</div>')


class Raw(str):
    """Cell text that is already HTML."""


def esc(x):
    return x if isinstance(x, Raw) else html.escape(str(x))


def table(rows, head=None, cls=""):
    h = "" if head is None else "<tr>" + "".join(f"<th>{esc(x)}</th>" for x in head) + "</tr>"
    body = "".join("<tr>" + "".join(f"<td>{esc(x)}</td>" for x in r) + "</tr>" for r in rows)
    return f'<table class="{cls}">{h}{body}</table>'


def hole_groups(holes):
    """One line per kind of hole: count, thread, depth, route, ids."""
    groups = {}
    for h in holes:
        t = f"{h['thread']} {h['threadClass']}" if h["thread"] else f"dia {fmt(h['diameter'])}"
        key = (t, h["depth"], h["extent"], h["route"].split(" (")[0])
        groups.setdefault(key, []).append(h["id"])
    out = []
    for (t, depth, extent, route), ids in groups.items():
        label = ids[0] if len(ids) == 1 else f"{ids[0]} ... {ids[-1]}"
        out.append(f"{len(ids)} x {t} x {fmt(depth)} {extent}, {route} ({label})")
    return out


def page(drawing, title, sub, body, n, total):
    return (f'<section class="page"><div class="hd"><b>{html.escape(drawing)}</b> {html.escape(title)}'
            f'<span>{html.escape(sub)}</span></div>{body}'
            f'<div class="ft">{html.escape(COMPANY)} · joao@materia-expedita.pt · issued {date.today().isoformat()} · generated from '
            f'our machine model · the STEP files are authoritative for shape · sheet {n}/{total}</div></section>')


def pages(kind, data, sm, views, layout, cuts):
    c = data["casting"]
    d = CAST[kind]["drawing"]
    title = CAST[kind]["title"]
    out = []
    bnd = size(c["bounds"])
    prow = [(i["id"], i["role"], i["label"], "us (option: you make)" if i["origin"]["kind"].startswith("made") else "us (bought)",
             " x ".join(fmt(v) for v in insert_size(i)), i["material"] if i["origin"]["kind"].startswith("made")
             else i["origin"]["manufacturer"] + " " + i["origin"]["reference"] + ", 1.4571",
             ", ".join(fmt(v, 2) for v in centre(i["bounds"])), axis_name(i["mountNormal"]), i["finish"],
             fmt(i["positionToleranceMm"], 3), len(i["anchors"]), os.path.basename(i["step"]))
            for i in data["inserts"]]
    cover = f"""<div class="cols"><div>
<h2>Scope</h2>
<p>Please quote this casting as a finished mineral (epoxy granite) casting with the steel inserts and fixing sockets cast
in, located as drawn. <b>Inserts and fixing sockets are supplied to you by us, free of charge</b> (made or bought by us).
<b>Option:</b> price separately for making the C45 inserts yourself from the insert STEP files; the PFEIFER fixing
sockets remain bought parts. Steel mounting faces are ground or replicated after cure; the mineral is never machined.</p>
<h2>Casting data (from the model)</h2>
{table([("Outline (casting body), x y z", " x ".join(fmt(v, 2) for v in bnd) + " mm"),
        ("Outline with proud steel", " x ".join(fmt(v, 2) for v in sm["outline"]["withProudSteelMm"]) + " mm"),
        ("Volume", f"{c['volumeMm3'] / 1e6:.3f} L ({c['volumeMm3']:.0f} mm3)"),
        ("Density used", f"{fmt(c['density'], 0)} kg/m3 (assumed, model material 'mineral cast')"),
        ("Mass of the casting body", Raw(f'<b data-mass="{c["massKg"]:.6f}">{c["massKg"]:.2f} kg</b>')),
        ("Steel (inserts, sockets, anchors)", f"{sm['steelMassKg']:.2f} kg"),
        ("Cast assembly mass", f"{sm['castAssemblyMassKg']:.2f} kg"),
        ("Centre of mass (casting frame)", ", ".join(fmt(v, 1) for v in c["centreOfMass"]) + " mm"),
        ("Inserts (distinct STEP files)", Raw(f'<span data-insert-count="{len(data["inserts"])}">{len(data["inserts"])} placed '
         f'({len(steel_inserts(data))} steel, {len(sockets(data))} fixing sockets), {len(data["insertSteps"])} STEP files</span>')),
        ("Cores and cavities", f"{len(data['cavities'])} cavities, {sum(x['volumeMm3'] for x in data['cavities']) / 1e6:.2f} L "
                               "in total (overlaps not netted)"),
        ("Lifting points", f"{len(data['liftingPoints'])} placeholder M12 threads, not qualified (hardware deferred)")]
       , None, "kv")}
</div><div>
<h2>Files</h2>
{table([("STEP", f"{c['steps']['body']}: casting body as cast (inserts cut out)"),
        ("STEP", f"{c['steps']['withInserts']}: casting with every cast-in insert, socket and anchor bolt in place (full)"),
        ("STEP", f"{c['steps']['withInsertsLight']}: same without anchor bolts (light)"),
        ("STEP", f"inserts/: {len(data['insertSteps'])} files, one per distinct insert, as supplied to the pour")]
       + [("", f"{s['step']}: x{s['quantity']} ({s['firstPlacement']}...)") for s in data["insertSteps"]], None)}
<h2>Frame</h2><p>{html.escape(data['units'])}.</p>
<h2>Sheets</h2><p>1 summary · 2-3 face views with overall dimensions · 4-5 insert layout and datum faces · 6-7 insert and hole schedules ·
8+ sections · cores and cavities · tolerances, notes and open questions.</p>
</div></div>"""
    out.append(("Summary", cover))
    out.append(("General arrangement", '<div class="grid">' + "".join(cell(views[n], 197, 118) for n in ("face-z-max", "face-y-min", "face-x-min"))
                + f'<div class="cell" style="width:197mm;height:118mm"><div class="vt">Overall dimensions</div>'
                  f'{table([("x", fmt(bnd[0], 2) + " mm"), ("y", fmt(bnd[1], 2) + " mm"), ("z", fmt(bnd[2], 2) + " mm")], None, "kv")}'
                  f'<p>Faces are sections taken 0.5 mm inside each face, so they show exactly what opens onto the face. Steel '
                  f'inserts are cast in with the mounting faces standing proud where drawn on the layout sheets.</p></div></div>'))
    out.append(("Other faces", '<div class="grid">' + "".join(cell(views[n], 197, 118) for n in ("face-z-min", "face-y-max", "face-x-max")) + "</div>"))
    out.append(("Insert layout, plan", cell(layout["top"], 396, 232)))
    out.append(("Insert layout, elevations", '<div class="grid">' + cell(layout["front"], 197, 232) + cell(layout["right"], 197, 232) + "</div>"))
    out.append(("Insert schedule",
                "<p>Insert frame: x and y from the casting centre, z up from the cast underside. Centre is the centre of the insert's "
                "bounding box. 'Mount normal' is the finished mounting plane normal. Red edges on the layout sheets are datum faces "
                "(ground or replicated after casting).</p>"
                + table(prow, ["Insert", "Role", "Label", "Supplied by", "Stock size", "Material / part", "Centre x, y, z", "Mount normal",
                               "Finish", "Pos tol mm", "Anchor bolts", "STEP"], "small")))
    hole_rows = []
    for i in steel_inserts(data):
        hole_rows.append((i["id"], "; ".join(hole_groups([h for h in i["holes"] if not h["id"].startswith("back-anchor")])) or "none",
                          f"{len([h for h in i['holes'] if h['id'].startswith('back-anchor')])} anchor holes (M8/M6 bolts)"))
    sock = sockets(data)
    srows = [(insert_mark(s["id"]), s["holes"][0]["thread"] + " " + s["holes"][0]["threadClass"], fmt(s["holes"][0]["depth"]),
              fmt(s["holes"][0]["threadDepth"] or 0), ", ".join(fmt(v, 2) for v in s["holes"][0]["entry"]),
              axis_name(s["holes"][0]["direction"]), fmt(s["positionToleranceMm"], 2)) for s in sock]
    body = "<h2>Threaded and plain holes in the steel inserts</h2>" + table(hole_rows, ["Insert", "Holes (route)", "Anchorage"], "small")
    if srows:
        body += ("<h2>Fixing sockets</h2><p>PFEIFER 05.250.064.040 (crimped end, M6 x 40, bore 25 deep, full thread 6, 1.4571), nominal study "
                 "geometry; bought by us and supplied to you. Entry is the open end of the socket, which sits at the finished surface.</p>"
                 + table(srows, ["Socket", "Thread", "Bore depth", "Full thread", "Entry x, y, z", "Axis into the socket", "Pos tol, mm"], "small"))
    out.append(("Holes and fixing sockets", body))
    per = 4
    for k in range(0, len(cuts), per):
        out.append((f"Sections {k // per + 1}", '<div class="grid">' + "".join(cell(v, 197, 118) for v in cuts[k:k + per]) + "</div>"))
    cats = [(k, v["count"], fmt(v["volumeL"], 3)) for k, v in sm["cavitiesByCategory"].items()]
    cav = ("<h2>Cores and cavities by group</h2>" + table(cats, ["Group", "Count", "Volume L"], "small")
           + "<p>Volumes are of each cavity inside the casting; overlapping cavities are not netted, so the sum is an upper bound. "
             "The table of every core follows.</p>")
    if kind == "bed":
        cav += (f"<h2>Coolant tank and foot pockets</h2><p>The casting reserves {fmt(c['coolantReservedLitres'], 2)} L for a separate coolant tank "
                "below and forward of the guide seats, with the chip drain and a front service opening (coolant tank access). The "
                "tank is a separate vessel; this figure is a model constant, not a measured cavity. Three levelling-foot pockets "
                "open from the sides and the rear (foot0, foot1: y sides; foot2: x max end); each takes a removable 110 x 110 x 15 mm "
                "ground plate (supplied by us) that bears on a replicated pocket roof.</p>"
                + table([(f"foot{k}", fmt(f['x'], 1), fmt(f['y'], 1), " x ".join(fmt(v) for v in f["plate"])) for k, f in enumerate(c["levellingFootPlates"])],
                        ["Foot pocket", "x", "y", "Plate mm"], "small"))
    lift = [(l["id"], l["insert"], l["hole"], l["thread"], fmt(l["depth"]), ", ".join(fmt(v, 2) for v in l["entry"]), axis_name(l["direction"]))
            for l in sm["liftingPoints"]]
    cav += ("<h2>Lifting points</h2><p><b>Not designed.</b> " + html.escape(sm["liftingNote"]) + "</p>"
            + table(lift, ["Point", "Plate", "Hole", "Thread", "Depth", "Entry x, y, z", "Axis into the plate"], "small"))
    out.append(("Cores, cavities, lifting", cav))

    def faces(cv):
        f = [o.replace("body.", "") for o in cv["openings"] if o.startswith("body.")]
        links = len(cv["openings"]) - len(f)
        return ", ".join(f) + (f" (+{links} linked cores)" if links else "") if (f or links) else "closed"

    crow = [(cv["id"], cv["category"].split(" (")[0], faces(cv), fmt(cv["volumeMm3"] / 1e6, 3),
             " x ".join(fmt(b - a, 1) for a, b in zip(cv["min"], cv["max"])), ", ".join(fmt(v, 1) for v in cv["min"]),
             fmt(cv["draftDegrees"], 1)) for cv in sm["cavities"]]
    chunk = 30
    for k in range(0, len(crow), chunk):
        out.append((f"All cores and cavities {k // chunk + 1}/{(len(crow) + chunk - 1) // chunk}",
                    "<p>Box = bounding box of each cavity (dx x dy x dz, then its minimum corner). Draft is as in the model: none is "
                    "modelled; mould draft is yours to propose.</p>"
                    + table(crow[k:k + chunk], ["Feature", "Group", "Opens to faces", "Volume L", "Box dx x dy x dz", "Box min x, y, z",
                                                "Draft deg"], "small")))
    trows = [(a, b, s) for a, b, s in tolerance_rows(kind, data)]
    notes_html = "<ol>" + "".join(f"<li>{html.escape(x)}</li>" for x in notes(kind, data)) + "</ol>"
    out.append(("Tolerances and notes", "<h2>Tolerances</h2><p>'Model' rows are numbers the model holds; every other row is "
                + AGREE + ".</p>" + table(trows, ["Feature", "Requirement", "Source"], "small") + "<h2>Notes</h2>" + notes_html))
    out.append(("Open questions", "<h2>What a foundry will ask and the model does not answer</h2><ol>"
                + "".join(f"<li>{html.escape(g)}</li>" for g in GAPS) + "</ol>"))
    total = len(out)
    return [page(d, f"{title}: {name}", f"{data['source'].split(' (')[0]}", body, n + 1, total) for n, (name, body) in enumerate(out)]


CSS = """ @page { size: A3 landscape; margin: 10mm; }
 body { font-family: Helvetica, Arial, sans-serif; font-size: 8.5pt; color: #111; margin: 0; }
 .page { page-break-after: always; width: 400mm; height: 274mm; position: relative; overflow: hidden; }
 .hd { font-size: 12pt; border-bottom: 1.5px solid #000; padding-bottom: 1mm; margin-bottom: 2mm; } .hd span { float: right; font-size: 8pt; color: #555; }
 .ft { position: absolute; bottom: 0; left: 0; right: 0; font-size: 7pt; color: #555; border-top: 1px solid #aaa; padding-top: 1mm; }
 .grid { display: flex; flex-wrap: wrap; gap: 2mm 4mm; } .cell { overflow: hidden; border: 1px solid #ccc; box-sizing: border-box; }
 .vt { font-weight: bold; font-size: 8pt; background: #eef2f6; padding: 1mm 2mm; height: 5mm; } .vt span { font-weight: normal; color: #555; }
 h2 { font-size: 10pt; margin: 3mm 0 1mm; } table { border-collapse: collapse; width: 100%; margin-bottom: 2mm; }
 th, td { border: 1px solid #bbb; padding: 1.5px 4px; text-align: left; vertical-align: top; } th { background: #eef2f6; }
 .small td, .small th { font-size: 7.5pt; } .kv td:first-child { width: 42%; background: #f6f8fa; }
 .cols { display: flex; gap: 8mm; } .cols > div { flex: 1; } svg { display: block; } p, li { margin: 1mm 0; } """


# ---------------------------------------------------------------------------------------------------------------
# Drawing-versus-JSON checks
# ---------------------------------------------------------------------------------------------------------------
def close(a, b, t=2e-3):
    return abs(a - b) <= t


def check_drawings(kind, data, doc, dxf_path, place, views, layout, cuts):
    c = data["casting"]
    # Mass printed on the summary sheet.
    m = re.search(r'data-mass="([0-9.]+)"', doc)
    if not m or not close(float(m.group(1)), c["massKg"], 1e-5):
        sys.exit(f"{kind}: mass on the drawing is {m and m.group(1)}, JSON says {c['massKg']}")
    # Insert count printed.
    m = re.search(r'data-insert-count="(\d+)"', doc)
    if not m or int(m.group(1)) != len(data["inserts"]):
        sys.exit(f"{kind}: insert count on the drawing differs from the JSON ({len(data['inserts'])})")
    # Insert rectangles in the HTML: in each layout view, one per insert, at the JSON position.
    for orient, v in layout.items():
        found = {}
        for tag in re.findall(r"<polygon [^>]*>", doc):
            if f'data-view="{v.id}"' in tag and 'data-layer="INSERTS"' in tag:
                ident = re.search(r'data-insert="([^"]+)"', tag).group(1)
                found[ident] = [float(x) for x in re.search(r'data-bounds="([^"]+)"', tag).group(1).split(",")]
        if sorted(found) != sorted(i["id"] for i in data["inserts"]):
            sys.exit(f"{kind}: {v.id} draws inserts {sorted(found)}, JSON has {sorted(i['id'] for i in data['inserts'])}")
        for i in data["inserts"]:
            want = v.box(i["bounds"])
            if not all(close(a, b) for a, b in zip(found[i["id"]], want)):
                sys.exit(f"{kind}: {v.id} insert {i['id']} drawn at {found[i['id']]}, JSON puts it at {want}")
        datums = set(re.findall(r'data-view="%s"[^>]*data-datum="([^"]+)"' % re.escape(v.id), doc))
        datums |= set(re.findall(r'data-datum="([^"]+)"[^>]*data-view="%s"' % re.escape(v.id), doc))
        if datums != {i["datumFace"]["id"] for i in data["inserts"] if i["datumFace"]}:
            sys.exit(f"{kind}: {v.id} datum faces {sorted(datums)} differ from the JSON")
    # Outline dimensions in every view that dimensions it.
    for v in list(layout.values()) + [views[n] for n in FACE_VIEW]:
        want = {"h": abs(v.box(c["bounds"])[2] - v.box(c["bounds"])[0]), "v": abs(v.box(c["bounds"])[3] - v.box(c["bounds"])[1])}
        for name in ("h", "v"):
            vals = [float(x) for x in re.findall(r'data-dim="%s" data-value="([0-9.]+)" data-view="%s"' % (name, re.escape(v.id)), doc)]
            if len(vals) != 1 or not close(vals[0], want[name]):
                sys.exit(f"{kind}: {v.id} dimension {name} reads {vals}, outline is {want[name]}")
    # DXF read back.
    polys = read_dxf(dxf_path)
    for v in list(layout.values()) + [views[n] for n in FACE_VIEW] + cuts:
        ox, oy = place[v.id]
        out = polys.get(f"{v.id}_OUTLINE")
        if not out or len(out) != 1:
            sys.exit(f"{kind}: DXF layer {v.id}_OUTLINE missing")
        xs, ys = [p[0] - ox for p in out[0]], [p[1] - oy for p in out[0]]
        want = v.box(c["bounds"])
        if not all(close(a, b, 1e-3) for a, b in zip((min(xs), min(ys), max(xs), max(ys)), want)):
            sys.exit(f"{kind}: DXF outline of {v.id} differs from the casting bounds")
    for v in layout.values():
        ox, oy = place[v.id]
        rects = polys.get(f"{v.id}_INSERTS", [])
        if len(rects) != len(data["inserts"]):
            sys.exit(f"{kind}: DXF {v.id} has {len(rects)} inserts, JSON {len(data['inserts'])}")
        got = sorted(tuple(round(x, 2) for x in (min(p[0] - ox for p in r), min(p[1] - oy for p in r),
                                                  max(p[0] - ox for p in r), max(p[1] - oy for p in r))) for r in rects)
        want = sorted(tuple(round(x, 2) for x in v.box(i["bounds"])) for i in data["inserts"])
        if not all(all(close(a, b, 0.011) for a, b in zip(g, w)) for g, w in zip(got, want)):
            sys.exit(f"{kind}: DXF {v.id} insert rectangles differ from the JSON bounds")
    # Section views: the contours drawn are the exported ones.
    for v, sec in zip(cuts, [s for s in data["sections"] if s["id"].startswith("cut-")]):
        n = len([it for it in v.items if it["t"] == "poly" and it["layer"] == "SECTION"])
        if n != len(sec["contours"]) or len(polys.get(f"{v.id}_SECTION", [])) != n:
            sys.exit(f"{kind}: {v.id} contour count differs from the export")


# ---------------------------------------------------------------------------------------------------------------
def main():
    if len(sys.argv) < 2 or any(k not in CAST for k in sys.argv[2:]):
        sys.exit(__doc__)
    root, kinds = sys.argv[1], sys.argv[2:] or list(CAST)
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser") if shutil.which(c)), None)
    result = {"generated": date.today().isoformat(), "source": "materia machinekit examples/flexspline-hobber (FlexsplineHobber)",
              "company": COMPANY, "castings": {}}
    for kind in kinds:
        data = json.load(open(os.path.join(root, f"{kind}-package.json")))
        check_rows(kind, data, root)
        check_layout(kind, data)
        sm = summary(kind, data)
        views, layout, cuts = build_views(kind, data)
        order = [views[n] for n in FACE_VIEW] + list(layout.values()) + cuts
        dxf = os.path.join(root, f"{kind}-drawings.dxf")
        place = write_dxf(order, dxf)
        body = "".join(pages(kind, data, sm, views, layout, cuts))
        doc = (f'<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(CAST[kind]["title"])} drawings</title>'
               f"<style>{CSS}</style></head><body>{body}</body></html>")
        check_drawings(kind, data, doc, dxf, place, views, layout, cuts)
        hp = os.path.join(root, f"{kind}-drawings.html")
        with open(hp, "w") as f:
            f.write(doc)
        if chrome:
            subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                            f"--print-to-pdf={os.path.join(root, kind + '-drawings.pdf')}", "file://" + os.path.abspath(hp)],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            print("warning: no Chrome or Chromium on the PATH, PDF not written")
        os.remove(hp)
        result["castings"][kind] = sm
        print(f"{kind}: {sm['massKg']:.2f} kg, {len(data['inserts'])} inserts, {len(data['cavities'])} cavities, checks passed")
    with open(os.path.join(root, "casting-summary.json"), "w") as f:
        json.dump(result, f, indent=1)
        f.write("\n")


if __name__ == "__main__":
    main()
