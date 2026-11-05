#!/usr/bin/env python3
"""Supplier drawings for the hobber's three ground SFU1605 ball screws (X, Y, Z).

Writes, into the output directory, for each axis:
  ballscrew-<axis>.html/pdf   one A4 landscape sheet: side view with total/threaded length, nut,
                              end details (BK12 fixed end, BF12 floating end), notes, title block
  ballscrew-<axis>.dxf        side view 1:1 (R12, mm); layers OUTLINE, NUT, CENTER, DIM, TEXT

Usage: ballscrew_package.py <output-dir>

No Haxe run is needed. Values mirror the model; where it exports no value the constant is
hardcoded below with its source:
  lengths      FlexsplineHobber.hx: X = HobberXLayout.front().screwLength (260, LAYOUT_REPORT.json),
               Z = HobberZLayout.SHAFT_LENGTH (420), Y = HobberYLayout.SCREW_LENGTH (170)
  nut station  X = screwStart 450 - motorward nut origin 213.5 = 236.5; Z = HobberZLayout.NUT_STATION (312);
               Y = HobberYLayout.NUT_STATION (100). Station = nut body centre from the
               fixed-end face; the model's flange is on the floating-end side of the body
               (body 35 + flange 10, BallNut).
  nut          BallNut: body Ø28 x 35, flange Ø48 x 10, 4 x Ø5.5 (M5) on Ø38, preloaded.
  race         BallScrew.race(): Ø16, lead 5, 1 start, right hand. Ground (BallScrew.ground).
  end machining  HzMotionDrawingScrew (BK12 profile, HZMotion Motor_Bracket.pdf p10), which is
               the "required machining" of the model: fixed end 51 (Ø10 x 15 coupling journal,
               M12x1 x 12, Ø12 bearing seat to 51), floating end 11 (Ø10, DIN 471 groove
               1.15 wide, Ø9.6, 1.85..3 from the tip). Vallder's standard SFU1605 ends differ
               slightly (fixed end 54, 14 mm thread relief); the supplier's standard is accepted.
  supports     ScrewSupportCatalog / VallderSupportData: BK12 bore Ø12, BF12 bore Ø10.
Not in the model (assumed here): tolerances beyond "supplier standard"; the motor journal is a plain
Ø10 journal, no keyway (clamp coupling).
"""
import html
import os
import shutil
import subprocess
import sys
from datetime import date

COMPANY = "Materia Expedita, Rua das Oficinas Municipais 15, 2450-060 Nazare, Portugal"
NOTE_ENDS = "End machining may follow your standard for BK12/BF12; please send your drawing for approval."

# Race and nut (BallScrew.race(), BallNut).
RACE_D, LEAD = 16.0, 5.0
NUT_BODY_D, NUT_BODY_L, NUT_FLANGE_D, NUT_FLANGE_T, NUT_PCD, NUT_HOLE = 28.0, 35.0, 48.0, 10.0, 38.0, 5.5
# End machining (HzMotionDrawingScrew BK12 profile).
FIXED_END, COUPLING_D, COUPLING_L, THREAD_L, SEAT_D = 51.0, 10.0, 15.0, 12.0, 12.0
FLOAT_END, FLOAT_D, GROOVE_W, GROOVE_D, GROOVE_FROM_TIP = 11.0, 10.0, 1.15, 9.6, 3.0

AXES = {  # length, nut station (body centre from the fixed-end face)
    "X": dict(length=260.0, nut=200.0, note="Installed horizontally, X axis."),
    "Y": dict(length=170.0, nut=100.0, note="Installed horizontally, Y axis."),
    "Z": dict(length=420.0, nut=312.0, note="Installed vertically: fixed end (BK12) at the TOP, floating end (BF12) at the bottom."),
}


class Sheet:
    """Primitive collector, y up. Emits DXF R12 or SVG (y flipped, paper 297 x 210)."""

    def __init__(self, text_h):
        self.items, self.th = [], text_h

    def line(self, a, b, layer="OUTLINE", dash=None):
        self.items.append(("line", layer, a, b, dash))

    def text(self, p, s, layer="TEXT", h=None, anchor="l", bold=False):
        self.items.append(("text", layer, p, s, h or self.th, anchor, bold))

    def fillrect(self, x0, y0, x1, y1, layer="NUT"):
        self.items.append(("rect", layer, (x0, y0), (x1, y1)))

    def dxf(self):
        out = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]
        for it in self.items:
            if it[0] == "line":
                _, layer, a, b, _d = it
                out += ["0", "LINE", "8", layer, "10", f"{a[0]:.4f}", "20", f"{a[1]:.4f}", "30", "0.0",
                        "11", f"{b[0]:.4f}", "21", f"{b[1]:.4f}", "31", "0.0"]
            elif it[0] == "text":
                _, layer, p, s, h, anchor, _b = it
                w = 0.55 * h * len(s)
                x = p[0] - (w / 2 if anchor == "m" else w if anchor == "r" else 0)
                out += ["0", "TEXT", "8", layer, "10", f"{x:.4f}", "20", f"{p[1]:.4f}", "30", "0.0",
                        "40", f"{h:.3f}", "1", s]
            else:
                _, layer, a, b = it
                pts = [(a[0], a[1]), (b[0], a[1]), (b[0], b[1]), (a[0], b[1]), (a[0], a[1])]
                for p, q in zip(pts, pts[1:]):
                    out += ["0", "LINE", "8", layer, "10", f"{p[0]:.4f}", "20", f"{p[1]:.4f}", "30", "0.0",
                            "11", f"{q[0]:.4f}", "21", f"{q[1]:.4f}", "31", "0.0"]
        return "\n".join(out + ["0", "ENDSEC", "0", "EOF"]) + "\n"

    def svg(self):
        o = []
        for it in self.items:
            if it[0] == "line":
                _, layer, a, b, dash = it
                w = {"OUTLINE": 0.5, "NUT": 0.5, "CENTER": 0.18, "DIM": 0.18}.get(layer, 0.25)
                d = {"center": ' stroke-dasharray="6 1.2 1 1.2"', "dash": ' stroke-dasharray="1.6 1"'}.get(dash, "")
                o.append(f'<line x1="{a[0]:.3f}" y1="{210 - a[1]:.3f}" x2="{b[0]:.3f}" y2="{210 - b[1]:.3f}" '
                         f'stroke="#000" stroke-width="{w}"{d}/>')
            elif it[0] == "text":
                _, layer, p, s, h, anchor, bold = it
                a = {"l": "start", "m": "middle", "r": "end"}[anchor]
                wt = ' font-weight="bold"' if bold else ""
                o.append(f'<text x="{p[0]:.3f}" y="{210 - p[1]:.3f}" font-size="{h}" text-anchor="{a}"'
                         f'{wt}>{html.escape(s)}</text>')
            else:
                _, layer, a, b = it
                o.append(f'<rect x="{min(a[0], b[0]):.3f}" y="{210 - max(a[1], b[1]):.3f}" '
                         f'width="{abs(b[0] - a[0]):.3f}" height="{abs(b[1] - a[1]):.3f}" fill="#fff" stroke="#000" stroke-width="0.5"/>')
        return "\n".join(o)


def fmt(v):
    return f"{v:.2f}".rstrip("0").rstrip(".")


def steps(length):
    """Half-profile steps (z0, z1, radius) from the fixed-end face."""
    tip = length - FLOAT_END
    return [(0, COUPLING_L, COUPLING_D / 2), (COUPLING_L, FIXED_END - 24, 6.0), (FIXED_END - 24, FIXED_END, SEAT_D / 2),
            (FIXED_END, tip, RACE_D / 2), (tip, length - GROOVE_FROM_TIP, FLOAT_D / 2),
            (length - GROOVE_FROM_TIP, length - GROOVE_FROM_TIP + GROOVE_W, GROOVE_D / 2),
            (length - GROOVE_FROM_TIP + GROOVE_W, length, FLOAT_D / 2)]


def draw_view(sh, s, ox, oy, z0, z1, length, nut=None):
    """Side view of the window z0..z1 (mm along the screw) at scale s; centre line at oy."""
    X = lambda z: ox + (z - z0) * s
    Y = lambda r: oy + r * s
    hide = (nut - NUT_BODY_L / 2, nut + NUT_BODY_L / 2 + NUT_FLANGE_T) if nut is not None else None
    sh.line((X(z0) - 6, oy), (X(z1) + 6, oy), "CENTER", "center")
    prof = steps(length)
    for a, b, r in prof:
        a2, b2 = max(a, z0), min(b, z1)
        if b2 <= a2:
            continue
        pieces = [(a2, b2)]
        if hide and r == RACE_D / 2:  # shaft hidden inside the nut
            pieces = [(p, q) for p, q in ((a2, min(b2, hide[0])), (max(a2, hide[1]), b2)) if q > p]
        for p, q in pieces:
            for sg in (1, -1):
                sh.line((X(p), oy + sg * r * s), (X(q), oy + sg * r * s))
    for (a, b, r), (c, d, r2) in zip(prof, prof[1:]):
        if z0 <= b <= z1:
            for sg in (1, -1):
                sh.line((X(b), oy + sg * r * s), (X(b), oy + sg * r2 * s))
    for z, r in ((0, prof[0][2]), (length, prof[-1][2])):
        if z0 <= z <= z1:
            sh.line((X(z), oy - r * s), (X(z), oy + r * s))
    # M12x1 thread: root lines
    if z0 <= COUPLING_L and z1 >= COUPLING_L + THREAD_L:
        for sg in (1, -1):
            sh.line((X(COUPLING_L), oy + sg * 5.4 * s), (X(COUPLING_L + THREAD_L), oy + sg * 5.4 * s), "OUTLINE", "dash")
    if nut is not None and z0 <= hide[0] and hide[1] <= z1:
        b0, b1 = nut - NUT_BODY_L / 2, nut + NUT_BODY_L / 2
        sh.fillrect(X(b0), Y(-NUT_BODY_D / 2), X(b1), Y(NUT_BODY_D / 2))
        sh.fillrect(X(b1), Y(-NUT_FLANGE_D / 2), X(hide[1]), Y(NUT_FLANGE_D / 2))
    return X, Y


def dim_h(sh, x0, x1, y, text, ext_from=None, ext_to=None):
    """Horizontal dimension with extension lines from ext_from/ext_to (y values) to y."""
    t = sh.th * 0.5
    for x, e in ((x0, ext_from), (x1, ext_to)):
        if e is not None:
            sh.line((x, e), (x, y + (1.5 if y > e else -1.5)), "DIM")
    sh.line((x0, y), (x1, y), "DIM")
    for x in (x0, x1):
        sh.line((x - t, y - t), (x + t, y + t), "DIM")
    sh.text(((x0 + x1) / 2, y + 1.0), text, "DIM", anchor="m")


def leader(sh, p, q, text, anchor="l"):
    sh.line(p, q, "DIM")
    sh.text((q[0] + (0.8 if anchor == "l" else -0.8), q[1] - sh.th * 0.35), text, "DIM", anchor=anchor)


def main_view(sh, ax, ox, oy, s, th):
    L, n = ax["length"], ax["nut"]
    X, Y = draw_view(sh, s, ox, oy, 0, L, L, nut=n)
    up = dn = NUT_FLANGE_D / 2 * s + 8
    tl = L - FLOAT_END - FIXED_END
    dim_h(sh, X(FIXED_END), X(L - FLOAT_END), oy + up, f"threaded length {fmt(tl)}", oy + 9 * s, oy + 9 * s)
    dim_h(sh, X(0), X(L), oy - dn, f"total length {fmt(L)}", oy - 5 * s, oy - 5 * s)
    dim_h(sh, X(0), X(n), oy - dn + 4, f"nut centre {fmt(n)} from fixed end",
          oy - NUT_FLANGE_D / 2 * s - 0.5, oy - NUT_FLANGE_D / 2 * s - 0.5)
    sh.text((X(n + 5), oy + NUT_FLANGE_D / 2 * s + 1.2), "nut: flange towards floating end", anchor="m")


def details(sh, length, ox_f, ox_b, oy, s):
    X, Y = draw_view(sh, s, ox_f, oy, 0, 56, length)
    top = oy + 8 * s + 10
    dim_h(sh, X(0), X(COUPLING_L), top, fmt(COUPLING_L), oy + 5 * s, oy + 5 * s)
    dim_h(sh, X(COUPLING_L), X(COUPLING_L + THREAD_L), top, fmt(THREAD_L), oy + 6 * s, oy + 6 * s)
    dim_h(sh, X(FIXED_END - 24), X(FIXED_END), top, "24", oy + 6 * s, oy + 6 * s)
    dim_h(sh, X(0), X(FIXED_END), top + 8, f"{fmt(FIXED_END)} fixed end (face to race)", oy + 8 * s, oy + 8 * s)
    leader(sh, (X(7.5), oy - 5 * s), (X(1), oy - 8 * s - 9), "plain Ø10 journal, no keyway (clamp coupling)")
    leader(sh, (X(21), oy - 5.4 * s), (X(17), oy - 8 * s - 18), "M12x1 locknut thread, 12 long")
    leader(sh, (X(39), oy - 6 * s), (X(36), oy - 8 * s - 9), "Ø12 bearing seat (BK12)")
    leader(sh, (X(54), oy - 8 * s), (X(46), oy - 8 * s - 18), "Ø16 race")
    sh.text((ox_f, oy + 8 * s + 27), "Detail A: fixed end (BK12), motor side", bold=True)
    z0 = length - 24
    X, Y = draw_view(sh, s, ox_b, oy, z0, length, length)
    dim_h(sh, X(length - FLOAT_END), X(length), top, fmt(FLOAT_END) + " floating end", oy + 5 * s, oy + 5 * s)
    leader(sh, (X(length - 6), oy - 5 * s), (X(length - 22), oy - 8 * s - 9), "Ø10 bearing seat (BF12)")
    leader(sh, (X(length - 2.4), oy - 4.8 * s), (X(length - 4), oy - 8 * s - 18),
           f"groove {GROOVE_W} x Ø{GROOVE_D}, {fmt(GROOVE_FROM_TIP - GROOVE_W)} from tip", anchor="r")
    sh.text((ox_b, oy + 8 * s + 27), "Detail B: floating end (BF12)", bold=True)


def notes(ax):
    return [
        f"1. Ground ball screw SFU1605: Ø16, lead {LEAD:g} mm, 1 start, right hand. Lead accuracy class C5 or better (ground).",
        "2. Nut: preloaded flanged ball nut (no axial play); flange Ø48 x 10, 4 x Ø5.5 holes on Ø38 PCD (M5), body Ø28.",
        "3. Ends for fixed support BK12 (Ø12 bore, preloaded angular-contact pair) and floating support BF12 (Ø10 bore).",
        "4. Material, heat treatment, journal tolerances and chamfers per your standard for SFU1605 with BK12/BF12 ends.",
        "5. Nut is drawn at its position in the machine; you may fit it anywhere on the thread, flange orientation as shown.",
        "6. " + ax["note"],
        "7. " + NOTE_ENDS,
    ]


def sheet(name, ax):
    L = ax["length"]
    sh = Sheet(3.0)
    main_view(sh, ax, 22, 173, min(0.8, 250.0 / ax["length"]), 3.0)
    details(sh, L, 22, 215, 90, 2.5)
    nt = notes(ax)
    notes_svg = "".join(f'<text x="10" y="{170.5 + 5.2 * i}" font-size="2.9">{html.escape(t)}</text>' for i, t in enumerate(nt))
    tb = [("Title", f"Ball screw SFU1605 x {fmt(L)}, {name} axis, ground, C5"),
          ("Drawing no.", f"MEX-HOB-BS-{name}  rev A"),
          ("Company", COMPANY),
          ("Units / scale", "mm; views to scale as shown, details 2.5:1"),
          ("Date", date.today().isoformat()),
          ("Contact", "joao@materia-expedita.pt")]
    rows = "".join(f'<text x="188" y="{169.5 + 6.2 * i}" font-size="2.4" fill="#444">{html.escape(k)}</text>'
                   f'<text x="209" y="{169.5 + 6.2 * i}" font-size="{2.9 if i != 2 else 2.35}">{html.escape(v)}</text>'
                   f'<line x1="186" y1="{171 + 6.2 * i}" x2="292" y2="{171 + 6.2 * i}" stroke="#000" stroke-width="0.15"/>'
                   for i, (k, v) in enumerate(tb))
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(name)} ball screw</title>
<style>@page {{ size: 297mm 210mm; margin: 0 }} html,body {{ margin:0; padding:0 }} svg {{ display:block; font-family: Arial, Helvetica, sans-serif }}</style></head><body>
<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210">
<rect x="0" y="0" width="297" height="210" fill="#fff"/>
<rect x="5" y="5" width="287" height="200" fill="none" stroke="#000" stroke-width="0.6"/>
<line x1="5" y1="164" x2="292" y2="164" stroke="#000" stroke-width="0.4"/><line x1="185" y1="164" x2="185" y2="205" stroke="#000" stroke-width="0.4"/>
{sh.svg()}
{notes_svg}
{rows}
</svg></body></html>"""


def dxf_for(name, ax):
    sh = Sheet(3.5)
    main_view(sh, ax, 0, 0, 1.0, 3.5)
    sh.text((0, 50), f"MEX-HOB-BS-{name} rev A - Ball screw SFU1605 x {fmt(ax['length'])}, {name} axis, ground C5, lead 5 - mm", bold=True)
    sh.text((0, 44), COMPANY)
    for i, t in enumerate(notes(ax)):
        sh.text((0, -62 - 5.5 * i), t)
    sh.text((0, -62 - 5.5 * 7), "End details: see PDF (fixed end 51: Ø10x15, M12x1x12, Ø12 to 51; floating end 11: Ø10, groove 1.15 wide Ø9.6).")
    return sh.dxf()


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    chrome = next((shutil.which(c) for c in ("google-chrome", "chromium", "chromium-browser") if shutil.which(c)), None)
    for name, ax in AXES.items():
        base = os.path.join(out, f"ballscrew-{name}")
        with open(base + ".html", "w") as f:
            f.write(sheet(name, ax))
        with open(base + ".dxf", "w") as f:
            f.write(dxf_for(name, ax))
        if chrome:
            subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                            f"--print-to-pdf={base}.pdf", "file://" + os.path.abspath(base + ".html")],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"{name}: length {ax['length']:g}, nut station {ax['nut']:g}")


if __name__ == "__main__":
    main()
