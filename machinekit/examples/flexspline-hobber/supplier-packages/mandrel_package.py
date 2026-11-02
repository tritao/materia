#!/usr/bin/env python3
"""Build the supplier package for the hobber's expanding mandrel from
SupplierPackageExport's JSON (the same gear, hob and workholding the hobber uses).

Writes, into the output directory:
  mandrel-datasheet.html/pdf  part, requirements, interfaces, keep-out table, section
  mandrel-section.dxf         half section (r, z), mm 1:1: cup, sleeve and mandrel for datum A
                              (pads on the inside end face) and datum B (open-end stop land),
                              hob keep-out boundary, axis and seat line
  hob-keepout.csv             keep-out boundary: minimum radius reached by the hob at
                              each height below the seat

Usage: mandrel_package.py <supplier-package.json> <output-dir>

The keep-out is the swept hob body (a cylinder of the hob's outside diameter and
length) at full cutting depth, for every hob centre height the CAM job reaches
(hobCentreZMin upwards: engagement overrun plus axial clearance) and every swivel
angle within the exported range. The cup turns, so the envelope is the minimum radial
distance from the work axis. It covers the hob only: arbor, outboard support and the
rest of the head are checked in the machine model's collision audit.
"""
import html
import json
import math
import os
import shutil
import subprocess
import sys
from datetime import date

COMPANY = "Materia Expedita, Rua das Oficinas Municipais 15, 2450-060 Nazare, Portugal"
KEEPOUT_CLEARANCE = 0.3    # minimum radial clearance from the swept hob to the mandrel/fixture
# Workpiece bore and seat specification (Materia's flexspline drawing must match).
# H5 keeps bore tolerance + loading clearance within a practical sleeve expansion; the
# open-end bore edge only gets an edge break because the cup seats on a 0.3 mm land
# there, so the loading lead-in belongs on the mandrel nose.
BORE_TOLERANCE = "H5"
BORE_ROUNDNESS = 0.002
BORE_CYLINDRICITY = 0.003
BORE_RA = 0.8
BORE_EDGE = "edge break ≤ 0.1 mm (no chamfer: it would remove the seating contact)"
SEAT_FLATNESS = 0.002
SEAT_PERPENDICULARITY = 0.003
# Datum A (preferred): inside face of the closed end, 3 hardened pads on the mandrel nose.
PAD_R_IN, PAD_R_OUT = 24.5, 27.5   # pad contact ring, inside the cup's inside corner radius (about 1 mm)
PAD_HEIGHT = 1.0
SLEEVE_A_BELOW_EDGE = 3.0          # sleeve A starts this far below the open edge
SLEEVE_OD = 58.040                 # collapsed sleeve OD, -0.005/0 (diametral clearance 0.010-0.028 to the bore)


def keepout(d, z_step=0.25):
    """Minimum radial distance reached by the swept hob body at each height z <= 0."""
    g, h, w = d["gear"], d["hob"], d["workholding"]
    radius, half = h["outsideDiameter"] / 2, h["length"] / 2
    centre = g["rootDiameter"] / 2 + radius  # full depth: hob tip on the root circle
    samples = []  # (z offset of the point from the hob centre, radial distance)
    tilt_max = math.radians(w["swivelRangeDeg"])
    for i in range(31):
        th = -tilt_max + 2 * tilt_max * i / 30
        s, c = math.sin(th), math.cos(th)
        for j in range(81):
            t = -half + 2 * half * j / 80
            for k in range(1440):
                ph = 2 * math.pi * k / 1440
                for rho in ((radius,) if abs(t) < half else (radius, radius * 0.5, 0.0)):
                    x = centre + rho * math.cos(ph)
                    y = t * c - rho * math.sin(ph) * s
                    dz = t * s + rho * math.sin(ph) * c
                    samples.append((dz, math.hypot(x, y)))
    samples.sort()
    zc_min = w["hobCentreZMin"]
    profile = []
    z = 0.0
    while True:
        # Covered by a hob centre at zc >= zc_min: point offset dz = z - zc <= z - zc_min.
        limit = z - zc_min
        r = min((rad for dz, rad in samples if dz <= limit), default=math.inf)
        profile.append((z, r))
        if r > g["rootDiameter"] / 2 + radius * 0.9 or z < -60:
            break
        z -= z_step
    return profile


def write_csv(profile, path):
    with open(path, "w") as f:
        f.write("# Hob keep-out below the seat (z = 0, positive up). r_min: smallest radius the swept hob\n")
        f.write(f"# reaches at that height. Mandrel and fixture must stay inside r_min - {KEEPOUT_CLEARANCE} mm.\n")
        f.write("z_mm,r_min_mm,r_allowed_mm\n")
        for z, r in profile:
            f.write(f"{z:.3f},{r:.4f},{r - KEEPOUT_CLEARANCE:.4f}\n")


def section_shapes(d, profile):
    w = d["workholding"]
    rb, rbody, rblank = w["cupBore"] / 2, w["cupBodyOutside"] / 2, w["cupBlankOutside"] / 2
    band, height, end = w["toothBand"], w["cupHeight"], w["cupEnd"]
    cup = [(rb, 0), (rblank, 0), (rblank, band), (rbody, band), (rbody, height), (0, height),
           (0, height - end), (rb, height - end), (rb, 0)]
    rs, ls = SLEEVE_OD / 2, w["sleeveLength"]
    wall = 1.5
    # Datum B (alternative): sleeve from the seat, stop land on the open-end face.
    sleeve_b = [(rs - wall, 0), (rs, 0), (rs, ls), (rs - wall, ls), (rs - wall, 0)]
    rstop = w["stopDiameter"] / 2
    stop = [(rb - 1.0, -2.0), (rstop, -2.0), (rstop, 0), (rb - 1.0, 0)]
    top = w["protrusionAboveSeat"]
    body_b = [(0, -2.0), (rb - 1.0, -2.0), (rb - 1.0, 0), (rs - wall, 0), (rs - wall, top), (0, top), (0, -2.0)]
    # Datum A (preferred): sleeve starts 3 mm below the open edge, nose with 3 pads on the inside end face.
    za = height - end                       # inside face of the closed end
    sleeve_a = [(rs - wall, -SLEEVE_A_BELOW_EDGE), (rs, -SLEEVE_A_BELOW_EDGE), (rs, ls), (rs - wall, ls),
                (rs - wall, -SLEEVE_A_BELOW_EDGE)]
    nose = PAD_R_OUT + 0.5
    body_a = [(0, -6.0), (rs - wall, -6.0), (rs - wall, ls), (nose, ls), (nose, za - PAD_HEIGHT),
              (0, za - PAD_HEIGHT), (0, -6.0)]
    pads = [(PAD_R_IN, za - PAD_HEIGHT), (PAD_R_OUT, za - PAD_HEIGHT), (PAD_R_OUT, za), (PAD_R_IN, za),
            (PAD_R_IN, za - PAD_HEIGHT)]
    keep = [(r, z) for z, r in profile]
    allowed = [(r - KEEPOUT_CLEARANCE, z) for z, r in profile]
    # Margin of sleeve A to the keep-out limit where it hangs below the edge.
    margin = min(r - KEEPOUT_CLEARANCE - rs for z, r in profile if z >= -SLEEVE_A_BELOW_EDGE)
    return dict(cup=cup, sleeve_a=sleeve_a, sleeve_b=sleeve_b, stop=stop, body_a=body_a, body_b=body_b,
                pads=pads, keep=keep, allowed=allowed, margin=margin, za=za)


def write_dxf(shapes, path):
    def poly(layer, pts, closed):
        out = ["0", "POLYLINE", "8", layer, "66", "1", "70", "1" if closed else "0",
               "10", "0.0", "20", "0.0", "30", "0.0"]
        for x, y in pts:
            out += ["0", "VERTEX", "8", layer, "10", f"{x:.4f}", "20", f"{y:.4f}", "30", "0.0"]
        return out + ["0", "SEQEND", "8", layer]

    def line(layer, a, b):
        return ["0", "LINE", "8", layer, "10", f"{a[0]:.4f}", "20", f"{a[1]:.4f}", "30", "0.0",
                "11", f"{b[0]:.4f}", "21", f"{b[1]:.4f}", "31", "0.0"]
    zlow = min(z for _, z in shapes["keep"])
    out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC",
           "0", "SECTION", "2", "ENTITIES"]
    out += poly("CUP", shapes["cup"], True)
    out += poly("SLEEVE_A", shapes["sleeve_a"], True)
    out += poly("MANDREL_A", shapes["body_a"], True)
    out += poly("DATUM_A_PADS", shapes["pads"], True)
    out += poly("SLEEVE_B", shapes["sleeve_b"], True)
    out += poly("MANDREL_B", shapes["body_b"], True)
    out += poly("DATUM_B_LAND", shapes["stop"], False)
    out += poly("HOB_KEEPOUT", shapes["keep"], False)
    out += poly("MANDREL_MAX_ENVELOPE", shapes["allowed"], False)
    out += line("AXIS", (0, zlow - 5), (0, 45))
    out += line("SEAT", (0, 0), (40, 0))
    out += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def svg_section(shapes, w, variant):
    """variant A: whole cup, datum A. variant B: lower part only, datum B."""
    s = 4.0
    zhi = w["cupHeight"] if variant == "A" else 22.0
    pts = [p for k in ("cup", "keep") for p in shapes[k]]
    rmax = max(r for r, _ in pts) + 2
    zmin, zmax = min(z for _, z in pts) - 2, zhi + 2
    if variant == "B":
        zmin = -12.0
    width, height = rmax * s + 230, (zmax - zmin) * s + 10

    def P(r, z):
        return f"{20 + r * s:.1f},{5 + (zmax - z) * s:.1f}"

    def path(points, closed=False):
        return "M" + " L".join(P(r, z) for r, z in points) + (" Z" if closed else "")
    keep = [p for p in shapes["keep"] if p[1] >= zmin]
    allowed = [p for p in shapes["allowed"] if p[1] >= zmin]
    keep_fill = keep + [(rmax, keep[-1][1]), (rmax, 0)]
    lx = 20 + rmax * s + 8

    def label(r, z, text, colour):
        x, y = (float(v) for v in P(r, z).split(","))
        return (f'<line x1="{x}" y1="{y}" x2="{lx - 3}" y2="{y}" stroke="#999" stroke-width="0.7"/>'
                f'<circle cx="{x}" cy="{y}" r="2" fill="{colour}"/>'
                f'<text x="{lx}" y="{y + 4}" font-size="10.5" fill="{colour}">{text}</text>')
    mid = keep[len(keep) // 2]
    ls = w["sleeveLength"]
    if variant == "A":
        labels = [
            label(PAD_R_OUT - 1, shapes["za"] - 0.5, "datum A: 3 pads, inside end face", "#0b6b3a"),
            label(w["cupBodyOutside"] / 2 - 0.3, 28, f"cup wall {w['cupWall']} mm, bore Ø{w['cupBore']:.2f}", "#1d3b5c"),
            label(w["cupBlankOutside"] / 2 - 0.5, w["toothBand"] / 2, f"tooth band {w['toothBand']:.0f} mm (hobbed)", "#1d3b5c"),
            label(SLEEVE_OD / 2 - 0.7, ls - 6, f"sleeve A: {ls + SLEEVE_A_BELOW_EDGE:.0f} mm, starts "
                  f"{SLEEVE_A_BELOW_EDGE:.0f} mm below edge", "#8a6d00"),
            label(mid[0] + 1, mid[1], "hob keep-out (swept hob, red)", "#b03030"),
        ]
        shapes_svg = (f'<path d="{path(shapes["body_a"], True)}" fill="#ececec" stroke="#888"/>'
                      f'<path d="{path(shapes["sleeve_a"], True)}" fill="#f2c94c" stroke="#8a6d00"/>'
                      f'<path d="{path(shapes["pads"], True)}" fill="#2e9e5e" stroke="#0b6b3a"/>')
    else:
        labels = [
            label(w["stopDiameter"] / 2 - 0.2, -1, f"datum B: stop land Ø{w['stopDiameter']:.2f} max", "#444"),
            label(SLEEVE_OD / 2 - 0.7, ls - 6, f"sleeve B: {ls:.0f} mm from the seat", "#8a6d00"),
            label(mid[0] + 1, mid[1], "hob keep-out (red)", "#b03030"),
            label(w["cupBlankOutside"] / 2 - 0.5, w["toothBand"] / 2, "tooth band", "#1d3b5c"),
        ]
        shapes_svg = (f'<path d="{path(shapes["body_b"], True)}" fill="#ececec" stroke="#888"/>'
                      f'<path d="{path(shapes["stop"])}" fill="#d9d9d9" stroke="#444"/>'
                      f'<path d="{path(shapes["sleeve_b"], True)}" fill="#f2c94c" stroke="#8a6d00"/>')
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">
 <path d="{path(keep_fill, True)}" fill="#f6d5d5" stroke="none"/>
 <path d="{path(keep)}" fill="none" stroke="#b03030" stroke-width="1.2"/>
 <path d="{path(allowed)}" fill="none" stroke="#b03030" stroke-width="0.8" stroke-dasharray="4 3"/>
 {shapes_svg}
 <path d="{path(shapes['cup'], True)}" fill="#cfe0f2" fill-opacity="0.75" stroke="#1d3b5c" stroke-width="1.1"/>
 <line x1="{P(0, zmin).split(',')[0]}" y1="{P(0, zmin).split(',')[1]}" x2="{P(0, zmax).split(',')[0]}" y2="{P(0, zmax).split(',')[1]}" stroke="#888" stroke-dasharray="8 3 2 3"/>
 <line x1="{P(0, 0).split(',')[0]}" y1="{P(0, 0).split(',')[1]}" x2="{P(rmax, 0).split(',')[0]}" y2="{P(rmax, 0).split(',')[1]}" stroke="#666" stroke-width="0.6"/>
 {"".join(labels)}
</svg>"""


def table(section, rows):
    body = "".join(f"<tr><th>{html.escape(k)}</th><td>{html.escape(v)}</td></tr>" for k, v in rows)
    return f"<h2>{html.escape(section)}</h2><table>{body}</table>"


def sheet(d, profile, shapes):
    g, w = d["gear"], d["workholding"]
    keep_rows = [(f"z = {z:.0f} mm", f"hob reaches r = {r:.2f} mm → mandrel/fixture r ≤ {r - KEEPOUT_CLEARANCE:.2f} mm")
                 for z, r in profile if abs(z - round(z)) < 1e-9 and round(z) in (0, -4, -8, -12, -16, -20)]
    part = [
        ("Workpiece", "Flexspline of a strain wave gear: one-piece thin-walled steel cup, open end down"),
        ("Clamping bore", f"Ø{w['cupBore']:.2f} {BORE_TOLERANCE}, honed after hardening; roundness {BORE_ROUNDNESS} mm, "
                          f"cylindricity {BORE_CYLINDRICITY} mm over the clamped length, Ra {BORE_RA} µm (0.4 preferred)"),
        ("Inside end face (datum A)", f"Face inside the closed end, {w['cupEnd']:.0f} mm plate. Ground or turned flat, "
                                      f"perpendicular to the bore within {SEAT_PERPENDICULARITY} mm. Pads touch it "
                                      f"inside R {PAD_R_OUT}"),
        ("Open-end face (datum B)", f"Flatness {SEAT_FLATNESS} mm, perpendicular to the bore within "
                                    f"{SEAT_PERPENDICULARITY} mm. Bore edge: {BORE_EDGE}"),
        ("Bore depth", f"{w['cupHeight'] - w['cupEnd']:.0f} mm (closed end plate {w['cupEnd']:.0f} mm)"),
        ("Outside", f"Ø{w['cupBlankOutside']:.0f} over the tooth band before hobbing; body Ø{w['cupBodyOutside']:.2f}"),
        ("After hobbing", f"tip Ø{g['tipDiameter']:.2f}, root Ø{g['rootDiameter']:.2f}; module {g['module']}, "
                          f"{g['teeth']} teeth"),
        ("Wall / tooth band", f"{w['cupWall']} mm under the tooth roots; band {w['toothBand']:.0f} mm at the open end"),
        ("Height", f"{w['cupHeight']:.0f} mm"),
        ("Material", "40CrNiMoA (SNCM439), through-hardened 38-42 HRC"),
    ]
    req = [
        ("Clamping principle", "Hydraulic expansion sleeve in the bore under the tooth band. The sleeve must be "
                               "continuous (no slots): a slotted collet is not acceptable on a 0.6 mm wall. "
                               "Other precision expansion: please propose"),
        ("Axial location, A (preferred)", f"Datum is the inside face of the closed end. 3 hardened pads on the "
                                          f"mandrel nose (120°), contact ring inside R {PAD_R_OUT}, pad plane "
                                          f"perpendicular to the bore axis within {SEAT_PERPENDICULARITY} mm. The cup "
                                          f"hangs on the pads. The sleeve runs past the open edge: it starts about "
                                          f"{SLEEVE_A_BELOW_EDGE:.0f} mm below the edge and is {w['sleeveLength'] + SLEEVE_A_BELOW_EDGE:.0f} mm long. "
                                          f"Its OD (R {SLEEVE_OD / 2:.3f}) stays inside the hob keep-out with "
                                          f"{shapes['margin']:.2f} mm margin. No land at the open end"),
        ("Axial location, B (alternative)", f"Hardened, ground stop land on the open-end face, Ø{w['stopDiameter']:.2f} "
                                            f"maximum (0.3 mm wide, the hob passes just outside it); sleeve "
                                            f"{w['sleeveLength']:.0f} mm from the seat. Pull-down onto the land "
                                            "during clamping"),
        ("Your advice", "Which of A or B do you recommend? How do you locate flexsplines or other thin cups "
                        "axially in your practice? Please quote A. Quote B only if you recommend it"),
        ("Sleeve size", f"Collapsed sleeve Ø{SLEEVE_OD:.3f} −0.005/0, so the loading clearance is 0.010 to 0.028 mm "
                        f"(diametral) in the Ø{w['cupBore']:.2f} {BORE_TOLERANCE} bore. Nose lead-in for robot loading, "
                        "at least 0.5 mm radial, 15°"),
        ("Hob keep-out", "Below the open edge the mandrel and its base must stay inside the keep-out boundary "
                         "(table below, mandrel-section.dxf, hob-keepout.csv). Please send a STEP model of "
                         "your proposal so we can check it in our machine model"),
        ("Height", f"Datum A: mandrel nose about {w['cupHeight'] - w['cupEnd']:.0f} mm above the open edge "
                   f"(datum B: about {w['protrusionAboveSeat']:.0f} mm). Open edge {w['seatAboveTableFace']:.0f} mm "
                   "above the rotary table face: your mandrel plus our adapter fill this; please state your height "
                   "from mounting face to the open edge"),
        ("Holding", "In an oily bore: torque ≥ 3.0 N·m and axial pull-off ≥ 28 N, at a contact pressure of "
                    "0.5 to 1.5 MPa over the whole loading-clearance range. Please state the oil pressure you "
                    "need for that; it is not the contact pressure"),
        ("Pressure zone", f"Contact pressure must act from the open edge (or below it) to at least 8 mm beyond the "
                          f"{w['toothBand']:.0f} mm tooth band"),
        ("Part distortion", f"We cut the teeth while the part is clamped. The {w['cupWall']} mm wall grows about "
                            "6.8 µm per MPa of contact pressure. Uniform radial expansion up to 0.010 mm is "
                            "acceptable, but you must state it and it must repeat (we compensate it by hob "
                            "depth). Lobed (non-uniform) distortion ≤ 0.0015 mm. Expansion variation over the "
                            f"{w['toothBand']:.0f} mm tooth band ≤ 0.003 mm. Clamped sleeve roundness ≤ 0.0015 mm. "
                            "Please state contact pressure and part expansion"),
        ("Accuracy", "Clamped bore runout to the mounting pilot ≤ 0.003 mm TIR. Re-clamp repeatability ≤ 0.0015 mm "
                     "over 50 cycles. Tilt ≤ 0.002 mm over 8 mm"),
        ("Loads", "Gear hobbing, module 0.3. Cutting force: tangential 47 N, radial 17 N. Torque on the table "
                  "1.5 N·m. Table speed in cut about 15 rpm (300 rpm is only the table maximum)"),
        ("Actuation", "Powered and fail-safe: the part stays clamped if air is lost. The mandrel must not be "
                      "pressurised without a part. Preferred: 6 bar air through a rotary union in the table's Ø40 mm "
                      "centre bore, with an air-to-oil booster in the mandrel (about 16:1, oil volume about "
                      "0.1 cm³). Alternative: draw-bar or push-rod from below, through the Ø40 mm bore"),
        ("Air interface", "Supply arrives centrally from below; ports on the base face within a Ø30 mm circle: "
                          "clamp, unclamp (or spring return), seat check if air-based"),
        ("Seat check", "Confirm the part is seated before cutting. For datum A: air-gauge nozzles or sensors in "
                       "the nose pads. Please propose. For B the land is only 0.3 mm wide, so please propose a "
                       "method. We read a pressure or switch signal"),
        ("Mounting", "Your standard base flange and centring; we make the adapter to the rotary table"),
        ("Environment", "Neat cutting oil and fine chips: please describe sealing and wipers"),
        ("Inspection", "Runout report with the mandrel, measured on a Ø58.05 master ring"),
        ("Life", "Automatic (robot) loading: please state the rated clamping cycles"),
        ("Quantity", "1 complete mandrel and 1 spare sleeve"),
    ]
    sweep = (f"Keep-out: swept hob body Ø{d['hob']['outsideDiameter']:.0f} × {d['hob']['length']:.0f} mm at full depth "
             f"(tip on the root circle), hob centre down to {-w['hobCentreZMin']:.2f} mm below the seat (cutting "
             f"overrun plus {w['camClearance']:.0f} mm clearance, from our CAM program), swivel "
             f"±{w['swivelRangeDeg']:.0f}°, minimum clearance {KEEPOUT_CLEARANCE} mm. Red: swept hob; dashed: mandrel "
             f"limit. The keep-out covers the hob; we check the arbor and support bracket against your design in our "
             f"machine model, so please send a STEP model with the quote.")
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>Expanding mandrel data sheet</title>
<style>
 @page {{ size: A4; margin: 14mm; }}
 body {{ font-family: Helvetica, Arial, sans-serif; font-size: 9.2pt; color: #111; }}
 h1 {{ font-size: 15pt; margin: 0 0 2mm; }} .sub {{ color: #555; margin-bottom: 3mm; }}
 h2 {{ font-size: 11.5pt; margin: 3mm 0 1.5mm; }}
 table {{ border-collapse: collapse; width: 100%; margin-bottom: 2mm; }}
 th, td {{ border: 1px solid #bbb; padding: 2.5px 6px; text-align: left; vertical-align: top; }}
 th {{ background: #eef2f6; width: 30%; font-weight: 600; }} .note {{ font-size: 8.5pt; color: #444; }}
</style></head><body>
<h1>Expanding mandrel data sheet: flexspline cup, bore Ø{w['cupBore']:.2f}</h1>
<div class="sub">{html.escape(COMPANY)} &middot; joao@materia-expedita.pt &middot; issued {date.today().isoformat()}</div>
{table("Workpiece", part)}{table("Mandrel requirements", req)}
{table("Hob keep-out below the seat (z = 0, positive up)", keep_rows)}
<h2>Half section (scale 4:1)</h2><b>Datum A (preferred)</b><br>{svg_section(shapes, w, "A")}
<br><b>Datum B (alternative), lower part</b><br>{svg_section(shapes, w, "B")}
<p class="note">{html.escape(sweep)} Files: mandrel-section.dxf (layers CUP, SLEEVE_A, MANDREL_A, DATUM_A_PADS, SLEEVE_B, MANDREL_B,
DATUM_B_LAND, HOB_KEEPOUT, MANDREL_MAX_ENVELOPE, AXIS, SEAT; r = x, z = y, mm; SEAT is the open edge) and hob-keepout.csv. Cup body dimensions are from our design
model; the production flexspline drawing will follow.</p>
</body></html>"""


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    data = json.load(open(sys.argv[1]))
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    profile = keepout(data)
    shapes = section_shapes(data, profile)
    write_csv(profile, os.path.join(out, "hob-keepout.csv"))
    write_dxf(shapes, os.path.join(out, "mandrel-section.dxf"))
    hp = os.path.join(out, "mandrel-datasheet.html")
    with open(hp, "w") as f:
        f.write(sheet(data, profile, shapes))
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser") if shutil.which(c)), None)
    if chrome:
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                        f"--print-to-pdf={os.path.join(out, 'mandrel-datasheet.pdf')}", "file://" + os.path.abspath(hp)],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    stop = data["workholding"]["stopDiameter"] / 2
    print(f"wrote {out}; keep-out at seat r={profile[0][1]:.3f}, stop land r={stop:.3f}, sleeve A margin {shapes['margin']:.3f}")


if __name__ == "__main__":
    main()
