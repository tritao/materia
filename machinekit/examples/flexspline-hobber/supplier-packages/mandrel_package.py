#!/usr/bin/env python3
"""Build the supplier package for the hobber's expanding mandrel from
SupplierPackageExport's JSON (the same gear, hob and workholding the hobber uses).

Writes, into the output directory:
  mandrel-datasheet.html/pdf  part, requirements, interfaces, keep-out table, section
  mandrel-section.dxf         half section (r, z), mm 1:1: cup, sleeve, stop land,
                              nominal mandrel, hob keep-out boundary, axis and seat line
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
    rs, ls = w["sleeveOutside"] / 2, w["sleeveLength"]
    sleeve = [(rs - 1.5, 0), (rs, 0), (rs, ls), (rs - 1.5, ls), (rs - 1.5, 0)]
    rstop = w["stopDiameter"] / 2
    stop = [(rb - 1.0, -2.0), (rstop, -2.0), (rstop, 0), (rb - 1.0, 0)]
    top = w["protrusionAboveSeat"]
    body = [(0, -2.0), (rb - 1.0, -2.0), (rb - 1.0, 0), (rs - 1.5, 0), (rs - 1.5, top), (0, top), (0, -2.0)]
    keep = [(r, z) for z, r in profile]
    allowed = [(r - KEEPOUT_CLEARANCE, z) for z, r in profile]
    return dict(cup=cup, sleeve=sleeve, stop=stop, body=body, keep=keep, allowed=allowed)


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
    out += poly("SLEEVE", shapes["sleeve"], True)
    out += poly("STOP_LAND", shapes["stop"], False)
    out += poly("MANDREL_NOMINAL", shapes["body"], True)
    out += poly("HOB_KEEPOUT", shapes["keep"], False)
    out += poly("MANDREL_MAX_ENVELOPE", shapes["allowed"], False)
    out += line("AXIS", (0, zlow - 5), (0, 45))
    out += line("SEAT", (0, 0), (40, 0))
    out += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def svg_section(shapes, w):
    s = 6.0
    pts = [p for k in ("cup", "keep") for p in shapes[k]]
    rmax = max(r for r, _ in pts) + 2
    zmin, zmax = min(z for _, z in pts) - 2, max(z for _, z in pts) + 2
    width, height = rmax * s + 230, (zmax - zmin) * s + 10

    def P(r, z):
        return f"{20 + r * s:.1f},{5 + (zmax - z) * s:.1f}"

    def path(points, closed=False):
        return "M" + " L".join(P(r, z) for r, z in points) + (" Z" if closed else "")
    keep_fill = shapes["keep"] + [(rmax, shapes["keep"][-1][1]), (rmax, 0)]
    lx = 20 + rmax * s + 8

    def label(r, z, text, colour):
        x, y = (float(v) for v in P(r, z).split(","))
        return (f'<line x1="{x}" y1="{y}" x2="{lx - 3}" y2="{y}" stroke="#999" stroke-width="0.7"/>'
                f'<circle cx="{x}" cy="{y}" r="2" fill="{colour}"/>'
                f'<text x="{lx}" y="{y + 4}" font-size="10.5" fill="{colour}">{text}</text>')
    labels = "".join([
        label(w["cupBodyOutside"] / 2 - 0.3, 28, f"cup wall {w['cupWall']} mm, bore Ø{w['cupBore']:.2f}", "#1d3b5c"),
        label(w["cupBlankOutside"] / 2 - 0.5, w["toothBand"] / 2, f"tooth band {w['toothBand']:.0f} mm (hobbed)", "#1d3b5c"),
        label(w["sleeveOutside"] / 2 - 0.7, w["sleeveLength"] - 6, f"clamping sleeve {w['sleeveLength']:.0f} mm", "#8a6d00"),
        label(w["stopDiameter"] / 2 - 0.2, -1, f"stop land Ø{w['stopDiameter']:.2f} max", "#444"),
        label(8, w["protrusionAboveSeat"] - 0.5, f"mandrel (nominal), top {w['protrusionAboveSeat']:.0f} mm above seat", "#666"),
        label(shapes["keep"][len(shapes["keep"]) // 2][0] + 1, shapes["keep"][len(shapes["keep"]) // 2][1],
              "hob keep-out (swept hob, red)", "#b03030"),
    ])
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">
 <path d="{path(keep_fill, True)}" fill="#f6d5d5" stroke="none"/>
 <path d="{path(shapes['keep'])}" fill="none" stroke="#b03030" stroke-width="1.2"/>
 <path d="{path(shapes['allowed'])}" fill="none" stroke="#b03030" stroke-width="0.8" stroke-dasharray="4 3"/>
 <path d="{path(shapes['body'], True)}" fill="#ececec" stroke="#888"/>
 <path d="{path(shapes['stop'])}" fill="#d9d9d9" stroke="#444"/>
 <path d="{path(shapes['sleeve'], True)}" fill="#f2c94c" stroke="#8a6d00"/>
 <path d="{path(shapes['cup'], True)}" fill="#cfe0f2" stroke="#1d3b5c" stroke-width="1.1"/>
 <line x1="{P(0, zmin).split(',')[0]}" y1="{P(0, zmin).split(',')[1]}" x2="{P(0, zmax).split(',')[0]}" y2="{P(0, zmax).split(',')[1]}" stroke="#888" stroke-dasharray="8 3 2 3"/>
 <line x1="{P(0, 0).split(',')[0]}" y1="{P(0, 0).split(',')[1]}" x2="{P(rmax, 0).split(',')[0]}" y2="{P(rmax, 0).split(',')[1]}" stroke="#666" stroke-width="0.6"/>
 {labels}
</svg>"""


def table(section, rows):
    body = "".join(f"<tr><th>{html.escape(k)}</th><td>{html.escape(v)}</td></tr>" for k, v in rows)
    return f"<h2>{html.escape(section)}</h2><table>{body}</table>"


def sheet(d, profile, shapes):
    g, w = d["gear"], d["workholding"]
    keep_rows = [(f"z = {z:.0f} mm", f"hob reaches r = {r:.2f} mm → mandrel/fixture r ≤ {r - KEEPOUT_CLEARANCE:.2f} mm")
                 for z, r in profile if abs(z - round(z)) < 1e-9 and (round(z) % 2 == 0 or z > -4)]
    part = [
        ("Workpiece", "Flexspline of a strain wave gear: one-piece thin-walled steel cup, open end down on the seat"),
        ("Clamping bore", f"Ø{w['cupBore']:.2f} {BORE_TOLERANCE}, honed after hardening; roundness {BORE_ROUNDNESS} mm, "
                          f"cylindricity {BORE_CYLINDRICITY} mm over the clamped length, Ra {BORE_RA} µm (0.4 preferred)"),
        ("Bore edge at the open end", BORE_EDGE),
        ("Seating end face", f"Open-end face: flatness {SEAT_FLATNESS} mm, perpendicular to the bore within "
                             f"{SEAT_PERPENDICULARITY} mm"),
        ("Bore depth", f"{w['cupHeight'] - w['cupEnd']:.0f} mm (closed end plate {w['cupEnd']:.0f} mm)"),
        ("Outside", f"Ø{w['cupBlankOutside']:.0f} over the tooth band before hobbing; body Ø{w['cupBodyOutside']:.2f}"),
        ("After hobbing", f"tip Ø{g['tipDiameter']:.2f}, root Ø{g['rootDiameter']:.2f}; module {g['module']}, "
                          f"{g['teeth']} teeth"),
        ("Wall / tooth band", f"{w['cupWall']} mm under the tooth roots; band {w['toothBand']:.0f} mm at the open end"),
        ("Height", f"{w['cupHeight']:.0f} mm"),
        ("Material", "40CrNiMoA (SNCM439), through-hardened 38-42 HRC"),
    ]
    req = [
        ("Clamping principle", "Expanding mandrel in the bore under the tooth band. Hydraulic expansion sleeve "
                               "preferred; precision mechanical expansion acceptable: please propose"),
        ("Sleeve", f"Clamping length {w['sleeveLength']:.0f} mm from the seat. Please size the sleeve and its "
                   f"expansion for the {BORE_TOLERANCE} bore with clearance for robot loading (our layout assumes "
                   f"about {(w['cupBore'] - w['sleeveOutside']) / 2:.3f} mm radial; less is acceptable), with a "
                   "lead-in chamfer on the mandrel nose"),
        ("Axial location", f"Hardened, ground stop land, Ø{w['stopDiameter']:.2f} maximum (the hob passes "
                           "just outside it); axial pull-down onto the land during clamping preferred"),
        ("Hob keep-out", "Below the seat the mandrel and its base must stay inside the keep-out boundary "
                         "(table below, mandrel-section.dxf, hob-keepout.csv). Please send a STEP model of "
                         "your proposal so we can check it in our machine model"),
        ("Height", f"Mandrel top about {w['protrusionAboveSeat']:.0f} mm above the seat (30 mm maximum). "
                   f"Seat {w['seatAboveTableFace']:.0f} mm above the rotary table face: your mandrel plus our "
                   "adapter fill this; please state your height from mounting face to seat"),
        ("Concentricity", "Clamped runout of the cup bore to the mounting pilot ≤ 0.003 mm TIR; re-clamp repeatability "
                         "≤ 0.0015 mm over 50 cycles; axis tilt ≤ 0.002 mm over 8 mm; stop land flat to 0.002 mm"),
        ("Holding", "Torque transmitted at the bore ≥ 3.0 N·m (2 × the 1.49 N·m cutting-torque bound) and "
                    "pull-off ≥ 28 N, at a contact pressure of 0.5 to 1.5 MPa over the whole loading-clearance "
                    "range. Please state the oil pressure you need for that; it is not the contact pressure"),
        ("Part distortion", f"The teeth are cut while the part is clamped. The {w['cupWall']} mm wall expands about "
                            "6.8 µm per MPa of contact pressure. Uniform expansion up to 0.010 mm radial is "
                            "acceptable (we compensate it by hob depth; repeatability ±0.001 mm). Non-uniform "
                            "(lobed, 2 or more lobes) clamped displacement ≤ 0.0015 mm; expansion variation over the "
                            f"{w['toothBand']:.0f} mm tooth band ≤ 0.003 mm; clamped sleeve OD roundness ≤ 0.0015 mm. "
                            "Contact pressure must start at the cup's open edge and extend at least 8 mm above the "
                            "tooth band. Please state contact pressure and part expansion"),
        ("Loads", "Gear hobbing, module 0.3: cutting force below 50 N, torque 1.5 N·m bound; table 15 rpm in cut"),
        ("Actuation (preferred)", "6 bar air through a rotary union in the table's Ø40 mm centre bore, "
                                  "air-to-hydraulic booster in the mandrel"),
        ("Actuation (alternative)", "Draw-bar or push-rod through the Ø40 mm table bore, cylinder under the "
                                    "table: please propose if this is your standard"),
        ("Air interface", "Supply arrives centrally from below; ports on the base face within a Ø30 mm circle: "
                          "clamp, unclamp (or spring return), seat check if air-based"),
        ("Seat check", "Confirm the part is seated before cutting. The stop land is only 0.3 mm wide, so "
                       "air-gauge holes cannot be in it: please propose a method (for example air through the "
                       "sleeve clearance before clamping, or a sensor at the closed end); we read a pressure or "
                       "switch signal"),
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
 body {{ font-family: Helvetica, Arial, sans-serif; font-size: 9.5pt; color: #111; }}
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
<h2>Half section (scale 6:1)</h2>{svg_section(shapes, w)}
<p class="note">{html.escape(sweep)} Files: mandrel-section.dxf (layers CUP, SLEEVE, STOP_LAND, MANDREL_NOMINAL, HOB_KEEPOUT,
MANDREL_MAX_ENVELOPE, AXIS, SEAT; r = x, z = y, mm) and hob-keepout.csv. Cup body dimensions are from our design
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
    print(f"wrote {out}; keep-out at seat r={profile[0][1]:.3f}, stop land r={stop:.3f}")


if __name__ == "__main__":
    main()
