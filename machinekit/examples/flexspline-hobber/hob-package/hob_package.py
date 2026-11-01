#!/usr/bin/env python3
"""Build the supplier hob package from HobPackageExport's JSON.

Writes, into the output directory:
  hob-rack-profile.csv   normal-section cutter tooth points (mm)
  hob-rack-profile.dxf   the same closed outline as a DXF R12 polyline (mm, 1:1)
  hob-datasheet.html/pdf hob ordering data
  gear-datasheet.html/pdf target gear data

Usage: hob_package.py <hob-package.json> <output-dir>
PDFs are printed with headless Chrome/Chromium when one is installed.
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


def write_csv(points, path):
    with open(path, "w") as f:
        f.write("# Hob normal-section cutter tooth (generating rack), mm.\n")
        f.write("# x: across the tooth, centred. y: height above the gear pitch line; "
                "the hob tooth tip is at the most negative y.\n")
        f.write("x_mm,y_mm\n")
        for x, y in points:
            f.write(f"{x:.6f},{y:.6f}\n")


def write_dxf(points, path, layer="RACK_TOOTH"):
    """Minimal DXF R12: one closed POLYLINE, millimetres, 1:1."""
    out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC",
           "0", "SECTION", "2", "ENTITIES",
           "0", "POLYLINE", "8", layer, "66", "1", "70", "1", "10", "0.0", "20", "0.0", "30", "0.0"]
    pts = points[:-1] if points[0] == points[-1] else points
    for x, y in pts:
        out += ["0", "VERTEX", "8", layer, "10", f"{x:.6f}", "20", f"{y:.6f}", "30", "0.0"]
    out += ["0", "SEQEND", "8", layer, "0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def svg_profile(points, hob, width=520, height=300):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    pad = 0.08
    minx, maxx = min(xs) - pad, max(xs) + pad
    miny, maxy = min(ys) - pad, max(ys) + pad
    scale = min(width / (maxx - minx), height / (maxy - miny))
    # Draw with the hob tip pointing down (y up = towards the hob root).
    def tx(x):
        return (x - minx) * scale

    def ty(y):
        return (maxy - y) * scale
    path = " ".join(f"{'M' if i == 0 else 'L'}{tx(x):.2f},{ty(y):.2f}" for i, (x, y) in enumerate(points))
    w, h = (maxx - minx) * scale, (maxy - miny) * scale
    pitch_y = ty(0.0)
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" viewBox="0 0 {w:.1f} {h:.1f}">
  <path d="{path} Z" fill="#dde6f0" stroke="#1d3b5c" stroke-width="1.2"/>
  <line x1="0" y1="{pitch_y:.1f}" x2="{w:.1f}" y2="{pitch_y:.1f}" stroke="#b03030" stroke-dasharray="6 4" stroke-width="1"/>
  <text x="4" y="{pitch_y - 4:.1f}" font-size="11" fill="#b03030">pitch line</text>
  <text x="4" y="{h - 6:.1f}" font-size="11" fill="#333">hob tooth tip (down)</text>
</svg>"""


def page(title, rows_html, extra=""):
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>
 @page {{ size: A4; margin: 16mm; }}
 body {{ font-family: Helvetica, Arial, sans-serif; font-size: 10.5pt; color: #111; }}
 h1 {{ font-size: 16pt; margin: 0 0 2mm; }}
 .sub {{ color: #555; margin-bottom: 5mm; }}
 table {{ border-collapse: collapse; width: 100%; margin-bottom: 4mm; }}
 th, td {{ border: 1px solid #bbb; padding: 3px 6px; text-align: left; vertical-align: top; }}
 th {{ background: #eef2f6; width: 42%; font-weight: 600; }}
 h2 {{ font-size: 12pt; margin: 5mm 0 2mm; }}
 .note {{ font-size: 9pt; color: #444; }}
</style></head><body>
<h1>{html.escape(title)}</h1>
<div class="sub">{html.escape(COMPANY)} &middot; joao@materia-expedita.pt &middot; issued {date.today().isoformat()}</div>
{rows_html}{extra}
</body></html>"""


def table(section, rows):
    body = "".join(f"<tr><th>{html.escape(k)}</th><td>{html.escape(v)}</td></tr>" for k, v in rows)
    return f"<h2>{html.escape(section)}</h2><table>{body}</table>"


def deg_min(deg):
    d = int(deg)
    m = (deg - d) * 60
    return f"{deg:.4f}° ({d}° {m:.1f}′)"


def span_measurement(g):
    """Base tangent length W_k for an external spur gear (ISO 21771)."""
    a = math.radians(g["pressureAngleDeg"])
    inv = math.tan(a) - a
    k = max(2, round(g["teeth"] * g["pressureAngleDeg"] / 180 + 0.5))
    w = g["module"] * math.cos(a) * (math.pi * (k - 0.5) + g["teeth"] * inv
                                     + 2 * g["profileShift"] * math.tan(a))
    return k, w


def hob_sheet(d):
    g, h = d["gear"], d["hob"]
    m = g["module"]
    rows_tool = [
        ("Quantity", "2 pcs"),
        ("Type", "Solid carbide monobloc gear hob, single thread, right hand"),
        ("Normal module", f"{m:.3f} mm"),
        ("Pressure angle", f"{g['pressureAngleDeg']:.1f}°"),
        ("Basic profile", f"Involute rack, hob addendum {h['addendum']:.3f} mm ({h['addendum'] / m:.2f} m), "
                          f"hob dedendum {h['dedendum']:.3f} mm ({h['dedendum'] / m:.2f} m), "
                          f"tooth depth {h['toothDepth']:.3f} mm"),
        ("Tip radius", f"{h['tipRadius']:.3f} mm ({h['tipRadius'] / m:.2f} m)"),
        ("Protuberance / tip chamfer", "None"),
        ("Normal pitch", f"{h['normalPitch']:.5f} mm"),
        ("Normal tooth thickness at pitch line", f"{h['normalPitch'] / 2:.5f} mm (no profile shift)"),
        ("Profile definition", "hob-rack-profile.dxf / .csv (normal section, 1:1, mm). "
                               "The profile is the standard involute rack above; files are for checking."),
    ]
    rows_body = [
        ("Outside diameter", f"{h['outsideDiameter']:.2f} mm"),
        ("Reference (pitch) diameter", f"{h['pitchDiameter']:.3f} mm"),
        ("Lead angle", deg_min(h["leadAngleDeg"])),
        ("Axial pitch / lead", f"{h['axialPitch']:.5f} mm / {h['lead']:.5f} mm"),
        ("Overall length", f"{h['length']:.0f} mm including hub collars (supplier standard collars for runout checking)"),
        ("Bore", f"Ø{h['bore']:.0f} mm H5 plain bore with one end drive slot (face key) to DIN 138, supplier standard slot dimensions; axially clamped on the arbor"),
        ("Gashes", f"{h['gashes']}, straight (parallel to axis)"),
        ("Rake angle", f"{h['rakeDeg']:.0f}°"),
        ("Relief", f"Ground (relieved) teeth; tip relief angle about {h['tipReliefDeg']:.0f}° (supplier to advise)"),
    ]
    rows_quality = [
        ("Accuracy class", "DIN 3968 class AA (or ISO 4468 AA)"),
        ("Substrate", "Fine-grain carbide (supplier to recommend grade)"),
        ("Coating", "Supplier to recommend (e.g. AlCrN / TiAlN) for neat-oil hobbing of 38-42 HRC steel"),
        ("Workpiece material", "40CrNiMoA (SNCM439), through-hardened 38-42 HRC before hobbing"),
        ("Inspection report", "Required with each hob: profile, pitch, lead, runout (DIN 3968)"),
        ("Services", "Please quote regrinding and recoating"),
    ]
    svg = svg_profile(d["rackTooth"], h)
    extra = (f"<h2>Normal-section tooth (generating rack)</h2>{svg}"
             f"<p class='note'>Profile generated from the gear design in gearkit (ConjugateRack); "
             f"{len(d['rackTooth'])} points in the CSV/DXF. Target gear: see gear-datasheet.pdf "
             f"(external spur, m {m}, z {g['teeth']}).</p>")
    return page("Gear hob data sheet: m 0.3 solid carbide hob",
                table("Tooth profile", rows_tool) + table("Hob body", rows_body)
                + table("Quality and material", rows_quality), extra)


def gear_sheet(d):
    g, h = d["gear"], d["hob"]
    k, w = span_measurement(g)
    rows = [
        ("Gear type", "External spur gear (flexspline of a strain wave gear: thin-walled cup, hobbed on an expanding mandrel)"),
        ("Normal module", f"{g['module']:.3f} mm"),
        ("Number of teeth", f"{g['teeth']}"),
        ("Pressure angle", f"{g['pressureAngleDeg']:.1f}°"),
        ("Helix angle", "0° (spur)"),
        ("Profile shift coefficient", f"{g['profileShift']:.3f}"),
        ("Reference diameter", f"{g['pitchDiameter']:.3f} mm"),
        ("Base diameter", f"{g['baseDiameter']:.3f} mm"),
        ("Tip diameter", f"{g['tipDiameter']:.3f} mm"),
        ("Root diameter", f"{g['rootDiameter']:.3f} mm"),
        ("Tooth thickness at reference circle", f"{g['pitchToothThickness']:.4f} mm (nominal, no backlash allowance)"),
        ("Span measurement", f"W{k} = {w:.4f} mm over {k} teeth (nominal)"),
        ("Root fillet radius (from hob tip)", f"{g['rootFilletRadius']:.3f} mm"),
        ("Tip edge rounding", f"{g['tipFilletRadius']:.3f} mm"),
        ("Face width", f"{g['faceWidth']:.1f} mm"),
        ("Accuracy target", "ISO 1328-1 class 5-6 equivalent (formula grades extrapolated below module 0.5)"),
        ("Material", "40CrNiMoA (SNCM439), through-hardened 38-42 HRC; cup wall about 0.6 mm"),
        ("Cutting", "CNC hobbing, neat cutting oil, one or two cuts to full depth"),
    ]
    return page("Gear data sheet: flexspline reference part", table("Gear data", rows),
                "<p class='note'>For the hob supplier's check of the hob design. Dimensions in mm.</p>")


def print_pdf(html_path, pdf_path):
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser")
                   if shutil.which(c)), None)
    if chrome is None:
        print("no Chrome/Chromium found; skipped", pdf_path)
        return
    subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    f"--print-to-pdf={pdf_path}", "file://" + os.path.abspath(html_path)],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    data = json.load(open(sys.argv[1]))
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    pts = [tuple(p) for p in data["rackTooth"]]
    write_csv(pts, os.path.join(out, "hob-rack-profile.csv"))
    write_dxf(pts, os.path.join(out, "hob-rack-profile.dxf"))
    for name, build in (("hob-datasheet", hob_sheet), ("gear-datasheet", gear_sheet)):
        hp = os.path.join(out, name + ".html")
        with open(hp, "w") as f:
            f.write(build(data))
        print_pdf(hp, os.path.join(out, name + ".pdf"))
    print("wrote package to", out)


if __name__ == "__main__":
    main()
