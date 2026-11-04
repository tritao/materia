#!/usr/bin/env python3
"""Hob spindle drawing package from the model export.

Usage: spindle_package.py <export dir>

Reads spindle-package.json (written by SpindlePackageExport.hx next to the per-part
STEP files) and writes, into the same directory:
  spindle-sections.dxf   exact axial sections of the made parts (DXF R12, mm, one
                          layer per part; x = axial from the housing rear datum, y = radial)
  spindle-drawings.html/.pdf  one overview page and one page per made part: section,
                          the model's diameters, and the tolerances and finishes we specify

Geometry comes only from the model. Tolerances, materials, heat treatment and
inspection are ordering requirements the model does not hold; they live here.
"""
import html
import json
import math
import os
import shutil
import subprocess
import sys
from datetime import date

COMPANY = "Materia Expedita, Rua das Oficinas Municipais 15, 2450-060 Nazaré, Portugal"
TOL = 1e-6

# Per made part: drawing number, title, material/treatment, and the toleranced features.
# Each feature row: (diameter or None, axial span or None, feature, requirement). A row with
# a diameter must match a cylinder of that diameter in the model section over that span.
PARTS = {
    "shaft": dict(
        drawing="HS-01", title="Spindle shaft", qty="1",
        material="38CrMoAlA (34CrAlNi7-10) quenched and tempered 28-32 HRC, stress relieved; gas nitrided "
                 "≥ 850 HV0.3, case 0.3 mm, on journals, taper and nose; ground after nitriding",
        rows=[
            (20, (-36, 0), "Pulley and encoder stub", "Ø20 h5 (encoder rotor 0 to -14, HTD5M 40T pulley -16 to -34, keyed or taper-bushed); radial runout 0.003 to A-B; Ra 0.4"),
            (25, (0, 15), "Rear journal (datum B), floating 6205", "Ø25 k5 (+0.011/+0.002); roundness 0.0015; Ra 0.4"),
            (25, (15, 122), "Between journals", "Ø25, may be relieved to Ø24.8; ISO 2768-m"),
            (None, (122, 129), "Locknut thread", "M25 × 1.5-4h for a KM5-size precision locknut; thread runs out before the front journal"),
            (25, (129, 163), "Front journal (datum A), matched angular-contact pair",
             "Ø25 +0.003/0, fitted to the bearing bores for 0-0.003 interference; roundness 0.001; "
             "cylindricity 0.0015; Ra 0.2"),
            (30, (163, 170), "Bearing shoulder", "face at z 163 perpendicular to A within 0.002; Ra 0.4"),
            (40, (170, 180), "Nose", "Ø40 h5 (labyrinth to the Ø42 gland bore); front face axial runout 0.002 "
                                     "to A-B, Ra 0.4"),
            (None, (160, 180), "Arbor taper socket", "1:5 on diameter, Ø20 at the face; AT3; contact ≥ 85 % "
                                                    "with the arbor (blue check); radial runout 0.002 to A-B; "
                                                    "Ra 0.2. Finish-grind in its own bearings preferred"),
            (None, None, "Drive pins", "2 × Ø4 m6 dowels pressed on Ø28 PCD (on the section plane), 2 mm proud"),
            (None, None, "Arbor screw holes", "4 × M4 × 10 deep on Ø32 PCD, at 45° to the pins"),
            (None, None, "Balance", "Rotor (shaft, encoder rotor, pulley, arbor) G1 at 4000 rpm"),
        ]),
    "housing": dict(
        drawing="HS-02", title="Spindle housing", qty="1",
        material="C45 (1.0503) normalised; stress relieved after rough machining. All bearing bores and "
                 "the outside register finished in one setting",
        rows=[
            (52, (0, 15), "Rear bearing bore, 6205 floating", "Ø52 H6 (+0.019/0) for axial float; coaxial to A "
                                                             "within 0.004; Ra 0.8"),
            (62, (15, 120), "Clearance bore", "Ø62, ISO 2768-m"),
            (44, (120, 129), "Shoulder bore", "Ø44, ISO 2768-m (clears the Ø38 locknut)"),
            (62, (129, 163), "Front bearing bore (datum A)", "Ø62 JS5 (±0.0065); roundness 0.002; cylindricity "
                                                             "0.002; Ra 0.4"),
            (62, (163, 180), "Gland spigot bore", "Ø62 H6"),
            (None, (129, 129), "Bearing shoulder face", "perpendicular to A within 0.002; Ra 0.8"),
            (80, (0, 180), "Outside register", "Ø80 g6, coaxial to A within 0.005 (seat for the machine's clamp ring and bracket)"),
            (None, None, "End faces", "front face perpendicular to A within 0.005; 4 × M4 × 10 deep on Ø72 PCD "
                                      "at 45°, both ends"),
        ]),
    "bearingRetainer": dict(
        drawing="HS-04", title="Front gland (bearing clamp and labyrinth)", qty="1",
        material="C45 (1.0503)",
        rows=[
            (62, (163, 180), "Spigot", "Ø62 g5 in the housing's H6 bore; end face at z 163 clamps the bearing "
                                       "outer rings: perpendicular within 0.003"),
            (80, (180, 186), "Flange", "4 × Ø4.5 on Ø72 PCD at 45°; grind the spigot length at assembly to "
                                       "leave 0.02-0.05 mm before the flange touches the housing"),
            (42, (163, 186), "Bore", "Ø42 H7 round the Ø40 nose: 1 mm radial labyrinth gap; add a flinger "
                                     "groove and drain at the bottom"),
        ]),
    "arbor": dict(
        drawing="HS-06", title="Hob arbor", qty="1 (+1 spare)",
        material="42CrMo4 (1.7225) hardened 52-56 HRC, or 1.2379 58-60 HRC; ground all over the functional "
                 "surfaces; stabilised",
        rows=[
            (None, (-20, 0), "Taper", "1:5 on diameter, Ø20 at the face; AT3; ≥ 85 % contact with the spindle "
                                      "socket; Ra 0.2"),
            (40, (0, 4), "Flange", "face perpendicular to the taper within 0.002; 2 × Ø4 H7 pin holes on Ø28; "
                                   "4 × Ø4.5 on Ø32 PCD at 45°"),
            (16, (4, 153), "Hob seat", "Ø16 h4 (0/-0.005) for the Ø16 H5 hob bore; radial runout 0.002 to the "
                                       "taper; cylindricity 0.002; Ra 0.2"),
            (15, (153, 175), "Outboard journal", "Ø15 k5 for the 6002 inner ring of the floating outboard "
                                                "support; coaxial to the taper within 0.002"),
            (None, None, "Hob drive and clamping (to add)", "drive key for the hob's DIN 138 end slot on the "
                                                           "flange side, ground spacer sleeve, and an M16 × 1 "
                                                           "clamp nut: dimensions follow the hob maker's slot drawing"),
        ]),
}
BOUGHT = {
    "motor": "Servo motor 1 kW, 80 frame, Ø19 × 35 shaft (Leadshine ELM2H-1000LA80F or equivalent); supplied by us, "
             "standing beside the cartridge on our cradle (not part of the cartridge)",
    "motorPulley": "HTD5M timing pulley 40T, 18 wide, bore Ø19 (ours, on the motor)",
    "spindlePulley": "HTD5M timing pulley 40T, 18 wide, bore Ø20 (on the cartridge stub; to be supplied or specified by the maker)",
    "belt": "HTD5M timing belt, 15 wide, 450 mm (90 teeth, 450-5M); ours",
    "motorEncoder": "Motor's own encoder (ours)",
    "frontBearing": "Angular-contact bearing 7305 size (25 × 62 × 17), P4, 25° contact; matched pair, back to back",
    "rearBearing": "Angular-contact bearing 7305 size (25 × 62 × 17), P4, 25° contact; matched pair, back to back",
    "tailBearing": "Deep groove ball bearing 6205 (25 × 52 × 15), P5 or better; floating",
    "encoder": "Absolute BiSS-C ring encoder, 19 bit, about Ø80 × 14 on the housing rear face (stator) and the shaft stub (rotor); the B master of our electronic gearbox; envelope assumed, maker to propose a catalogue part",
    "arborScrew": "Socket head cap screw M4 × 10, 12.9 (arbor flange)",
    "glandScrew": "Socket head cap screw M4 × 12 (gland)",
    "bearingLocknut": "Precision bearing locknut M25 × 1.5, KM5 size (Ø38 × 7), face runout ≤ 0.003, radial locking screws",
}
GENERAL = [
    "Dimensions in mm. Untoleranced: ISO 2768-mK. Break sharp edges 0.2 unless stated.",
    "Geometry is the nominal model: the STEP files are authoritative for shape. Threads are not modelled; "
    "tapped holes are modelled at the thread major diameter.",
    "Datums: A = front bearing journal/bore axis, B = rear journal/bore axis.",
    "Please state the inspection you provide (CMM report or runout/roundness measurements on the "
    "toleranced features).",
]
REQUIREMENTS = [
    ("Function", "Hob spindle for a gear hobbing machine, module 0.3, ISO class 5-6"),
    ("Speed", "up to 4000 rpm; normally 2000-3000 rpm; about 1 kW"),
    ("Running accuracy", "radial runout at the arbor taper and axial runout at the nose face ≤ 0.002 mm TIR"),
    ("Bearings", "front: matched P4 angular-contact pair 7305 size (25 × 62 × 17), back to back, light "
                 "preload; rear: 6205 floating; grease for life"),
    ("Drive", "belt-drive cartridge: HTD5M 40T pulley on the Ø20 rear stub, driven by a 1 kW servo (80 frame, "
              "Leadshine ELM2H-1000LA80F) on a 40T pulley (1:1) beside the cartridge; the belt's radial pull on the pulley is about "
              "450 N (quote the cartridge for at least 700 N radial at the pulley, 33 mm behind the 6205), "
              "motor torque up to 9.5 N·m at the spindle (1:1); a motor spindle is the upgrade path with the same seat and encoder interface"),
    ("Feedback", "19-bit absolute BiSS-C encoder on the rear face and stub: the machine's gearbox master"),
    ("Arbor", "removable from the front: 1:5 short taper and face, held by 4 × M4; 2 pins carry torque"),
    ("Coolant", "neat cutting oil, flood and mist: labyrinth at the nose"),
]


def section_cylinders(part):
    """Cylindrical surfaces in the section: (diameter, z0, z1) from segments at constant radius."""
    found = set()
    for contour in part["section"]:
        for s in contour["segments"]:
            (x0, y0), (x1, y1) = s["start"], s["end"]
            if s["kind"].lower() == "line" and abs(x0 - x1) < TOL and x0 > TOL:
                found.add((round(2 * x0, 4), round(min(y0, y1), 4), round(max(y0, y1), 4)))
    return sorted(found, key=lambda c: (c[1], c[0]))


def check_rows(part, rows):
    cyl = section_cylinders(part)
    for d, span, name, _ in rows:
        if d is None or span is None:
            continue
        lo, hi = span
        if not any(abs(c[0] - d) < 1e-3 and c[1] <= lo + 1e-3 and c[2] >= hi - 1e-3 for c in cyl):
            sys.exit(f"{part['id']}: '{name}' Ø{d} over z {lo}..{hi} is not a cylinder in the model section; "
                     f"model has {cyl}")


def segment_points(s, n=24):
    if s["kind"].lower() == "line":
        return [tuple(s["start"]), tuple(s["end"])]
    cx, cy = s["center"]
    a0 = math.atan2(s["start"][1] - cy, s["start"][0] - cx)
    r = math.hypot(s["start"][0] - cx, s["start"][1] - cy)
    return [(cx + r * math.cos(a0 + s["sweep"] * i / n), cy + r * math.sin(a0 + s["sweep"] * i / n))
            for i in range(n + 1)]


def contour_points(contour):
    pts = []
    for s in contour["segments"]:
        p = segment_points(s)
        pts.extend(p if not pts else p[1:])
    return pts


def placed(part, offset):
    """Section contours as (axial, radial) points, shifted axially into housing coordinates."""
    return [[(y + offset, x) for x, y in contour_points(c)] for c in part["section"]]


def svg(parts_contours, width_px=700, height_px=None, labels=()):
    pts = [p for contours in parts_contours.values() for c in contours for p in c]
    zmin, zmax = min(p[0] for p in pts) - 4, max(p[0] for p in pts) + 4
    rmax = max(abs(p[1]) for p in pts) + (10 if labels else 4)
    s = min(width_px / (zmax - zmin), (height_px or 1e9) / (2 * rmax))
    w, h = (zmax - zmin) * s, 2 * rmax * s

    def P(z, r):
        return f"{(z - zmin) * s:.2f},{(rmax - r) * s:.2f}"
    colours = ["#cfe0f2", "#f2e1c4", "#d9ecd0", "#ead1e8", "#f6d5d5", "#e3e3e3"]
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" viewBox="0 0 {w:.0f} {h:.0f}">']
    for i, (pid, contours) in enumerate(parts_contours.items()):
        d = " ".join("M" + " L".join(P(z, r) for z, r in c) + " Z" for c in contours)
        out.append(f'<path d="{d}" fill="{colours[i % len(colours)]}" fill-rule="evenodd" stroke="#222" '
                   f'stroke-width="0.8"/>')
    out.append(f'<line x1="0" y1="{rmax * s:.1f}" x2="{w:.0f}" y2="{rmax * s:.1f}" stroke="#888" '
               f'stroke-dasharray="10 3 2 3" stroke-width="0.7"/>')
    for z, r, text in labels:
        x, y = (float(v) for v in P(z, r).split(","))
        out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="9" fill="#333">{html.escape(text)}</text>')
    out.append("</svg>")
    return "".join(out), s


def write_dxf(data, path):
    out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC",
           "0", "SECTION", "2", "ENTITIES"]
    for part in data["made"]:
        layer = part["id"].upper()
        offset = 180.0 if part["id"] == "arbor" else 0.0
        for c in part["section"]:
            for s in c["segments"]:
                pts = segment_points(s)
                for a, b in zip(pts, pts[1:]):
                    out += ["0", "LINE", "8", layer, "10", f"{a[1] + offset:.4f}", "20", f"{a[0]:.4f}", "30", "0.0",
                            "11", f"{b[1] + offset:.4f}", "21", f"{b[0]:.4f}", "31", "0.0"]
    out += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def table(rows, head=None):
    h = "" if head is None else "<tr>" + "".join(f"<th>{html.escape(x)}</th>" for x in head) + "</tr>"
    body = "".join("<tr>" + "".join(f"<td>{html.escape(str(x))}</td>" for x in r) + "</tr>" for r in rows)
    return f"<table>{h}{body}</table>"


def member(data, mid):
    return next(m for m in data["members"] if m["id"] == mid)


def check_layout(data):
    """The fixed specifications above assume these member positions; fail if the model moved them."""
    lay = data["layout"]
    expect = {"tailBearing": 0.0, "frontBearing": 146.0, "rearBearing": 146.0,
              "arbor": lay["toolFace"], "encoder": 0.0}
    for mid, z in expect.items():
        got = member(data, mid)["position"][2]
        if abs(got - z) > 1e-6:
            sys.exit(f"model moved {mid} to z {got}, specification assumes {z}")


def pages(data):
    made = {p["id"]: p for p in data["made"]}
    lay, motor, arbor = data["layout"], data["motor"], data["arbor"]
    overview, _ = svg({pid: placed(made[pid], 180.0 if pid == "arbor" else 0.0)
                       for pid in ["housing", "bearingRetainer", "shaft", "arbor"]},
                      width_px=690, labels=[(-34, -50, "pulley z -16 to -34"), (1, -50, "rear datum z 0"),
                                            (112, -50, "bearing pair z 129-163"), (186, -50, "tool face z 180")])
    made_rows = [(PARTS[p["id"]]["drawing"], PARTS[p["id"]]["title"], PARTS[p["id"]]["qty"], p["step"],
                  f"{p['massKg']:.2f}") for p in data["made"]]
    made_ids = {p["id"] for p in data["made"]}
    # Supplier-facing names for the bought members, keyed by member id (prefix before a trailing number).
    bought, seen = [], {}
    for m in data["members"]:
        mid = m["id"].rstrip("0123456789")
        if m["id"] in made_ids or mid in ("motorShaft",):
            continue
        if mid not in BOUGHT:
            sys.exit(f"no supplier description for bought member {m['id']}")
        name = BOUGHT[mid]
        seen[name] = seen.get(name, 0) + 1
    bought = [(n, name) for name, n in seen.items()]
    first = f"""<section><h1>Hob spindle cartridge: drawing package</h1>
<div class="sub">{html.escape(COMPANY)} · joao@materia-expedita.pt · issued {date.today().isoformat()} ·
generated from our machine model</div>
<h2>Requirements</h2>{table(REQUIREMENTS)}
<h2>Assembly section (axial, through the drive pins)</h2>{overview}
<p class="note">Axial positions are from the housing rear datum (z 0) toward the tool face (z 180). Encoder
z 0 to {-lay['encoderLength']:.0f}; pulley (HTD5M {lay['pulleyTeeth']}T, motor {lay['motorTeeth']}T, {lay['centres']:.1f} mm centres)
z {lay['pulleyFront']:.0f} to {lay['pulleyFront'] - lay['pulleyWidth']:.0f}; Ø20 stub to z {lay['shaftRear']:.0f}. Servo (Leadshine ELM2H-1000LA80F or equivalent):
{motor['ratedTorque']:.2f} N·m, {motor['ratedRpm']:.0f}/{motor['maxRpm']:.0f} rpm, {motor['face']:.0f} frame,
Ø{motor['shaftDiameter']:.0f} × {motor['shaftLength']:.0f} shaft. Arbor {arbor['length']:.0f} long, hob seat
Ø{arbor['hobSeatDiameter']:.0f}, hob centre {arbor['hobCentre']:.0f} from the tool face.</p>
<h2>Made parts</h2>{table(made_rows, ["Drawing", "Part", "Qty", "STEP", "Mass kg"])}
<h2>Bought parts in the cartridge</h2>{table(bought, ["Qty", "Part"])}
<h2>General notes</h2><ul>{''.join(f'<li>{html.escape(g)}</li>' for g in GENERAL)}</ul></section>"""
    out = [first]
    for p in data["made"]:
        spec = PARTS[p["id"]]
        check_rows(p, spec["rows"])
        drawing, scale = svg({p["id"]: placed(p, 0.0)}, width_px=690, height_px=330)
        cyl = [(f"Ø{d:g}", f"{z0:g} to {z1:g}") for d, z0, z1 in section_cylinders(p)]
        frame = "gauge face z 0" if p["id"] == "arbor" else "housing rear datum z 0"
        out.append(f"""<section><h1>{spec['drawing']} {html.escape(spec['title'])}</h1>
<div class="sub">Qty {spec['qty']} · STEP {p['step']} · {p['massKg']:.3f} kg · axial section, {frame},
scale about {scale:.1f}:1 (dimension from the STEP)</div>{drawing}
<h2>Material and treatment</h2><p>{html.escape(spec['material'])}</p>
<h2>Specified features</h2>{table([(f'Ø{d:g}' if d else '', f'{s[0]:g} to {s[1]:g}' if s else '', n, r)
                                   for d, s, n, r in spec['rows']], ["Ø", "z", "Feature", "Requirement"])}
<h2>Cylinders in the model section</h2>{table(cyl, ["Diameter", "z from / to"])}</section>""")
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = sys.argv[1]
    data = json.load(open(os.path.join(out, "spindle-package.json")))
    check_layout(data)
    write_dxf(data, os.path.join(out, "spindle-sections.dxf"))
    body = "".join(pages(data))
    doc = f"""<!doctype html><html><head><meta charset="utf-8"><title>Hob spindle drawings</title><style>
 @page {{ size: A4; margin: 12mm; }}
 body {{ font-family: Helvetica, Arial, sans-serif; font-size: 9pt; color: #111; }}
 section {{ page-break-after: always; }} h1 {{ font-size: 14pt; margin: 0 0 1.5mm; }}
 h2 {{ font-size: 10.5pt; margin: 3mm 0 1.2mm; }} .sub {{ color: #555; margin-bottom: 2mm; }}
 table {{ border-collapse: collapse; width: 100%; margin-bottom: 1.5mm; }}
 th, td {{ border: 1px solid #bbb; padding: 2px 5px; text-align: left; vertical-align: top; }}
 th {{ background: #eef2f6; }} .note {{ font-size: 8pt; color: #444; }} svg {{ display: block; margin: 1mm 0; }}
</style></head><body>{body}</body></html>"""
    hp = os.path.join(out, "spindle-drawings.html")
    with open(hp, "w") as f:
        f.write(doc)
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser") if shutil.which(c)), None)
    if chrome:
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                        f"--print-to-pdf={os.path.join(out, 'spindle-drawings.pdf')}", "file://" + os.path.abspath(hp)],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"wrote {out}: {len(data['made'])} part pages")


if __name__ == "__main__":
    main()
