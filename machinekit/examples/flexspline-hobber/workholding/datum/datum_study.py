#!/usr/bin/env python3
"""Axial-datum comparison for the flexspline cup on the expanding mandrel (study, no model change).

Usage: datum_study.py <supplier-package.json> [figure.svg]
The JSON comes from supplier-packages/SupplierPackageExport.hx (see its README). The hob keep-out is
computed by supplier-packages/mandrel_package.py `keepout()` (swept hob, 30 swivel steps over +-15 deg,
0.3 mm clearance), so the numbers are the ones the mandrel supplier is given. stdlib only.

z is height above the cup's tooth-band bottom face for the cup, and "rel. seat" for the mandrel.
Current design: seat = band bottom = open end, the land is below it.
Option B: the seat is the collar's bottom face, the band bottom is Lc above the seat.
"""
import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "supplier-packages"))
import mandrel_package as mp  # noqa: E402

CLEAR = mp.KEEPOUT_CLEARANCE


def allowed_profile(d):
    """Allowed mandrel/fixture radius vs height rel. seat for the reference hob travel (numeric, with swivel)."""
    prof = mp.keepout(d, z_step=0.25)
    return [(z, r - CLEAR) for z, r in prof]


def shifted(base, zc_min0, zc_min):
    """Profile for a lowest hob centre at zc_min instead of zc_min0: the swept body only depends on z - zc."""
    return lambda z: at(base, z - zc_min + zc_min0)


def at(profile, z):
    """Linear interpolation of the allowed radius at height z (profile is sorted from z=0 downward)."""
    pts = sorted(profile)
    if z <= pts[0][0]:
        return pts[0][1]
    if z >= pts[-1][0]:
        return pts[-1][1]
    for (z0, r0), (z1, r1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            return r0 + (r1 - r0) * (z - z0) / (z1 - z0)


def closed_form_allowed(d, z, zc_min):
    """In-plane closed form (no swivel): hob circle of radius R centred at distance c from the work axis."""
    R = d["hob"]["outsideDiameter"] / 2
    c = d["gear"]["rootDiameter"] / 2 + R
    dz = z - zc_min
    if dz >= 0:
        return c - R - CLEAR
    return c - math.sqrt(max(R * R - dz * dz, 0.0)) - CLEAR


def main():
    d = json.load(open(sys.argv[1]))
    g, h, w = d["gear"], d["hob"], d["workholding"]
    R = h["outsideDiameter"] / 2
    root_r = g["rootDiameter"] / 2
    blank_r = w["cupBlankOutside"] / 2
    bore_r = w["cupBore"] / 2
    sleeve_r = w["sleeveOutside"] / 2
    band = w["toothBand"]
    cut_depth = blank_r - root_r
    chord = math.sqrt(R * R - (R - cut_depth) ** 2)
    zc_min = w["hobCentreZMin"]
    zc_max = w["hobCentreZMax"]
    print(f"hob R {R:.2f}, blank R {blank_r:.3f}, root R {root_r:.3f}, cut depth {cut_depth:.3f} mm")
    print(f"engagement chord (overrun) = sqrt(R^2-(R-d)^2) = {chord:.3f} mm; CAM clearance {w['camClearance']:.1f}")
    print(f"hob centre range rel. band bottom: {zc_min:.3f} .. {zc_max:.3f}  (check: -(chord+clearance) = "
          f"{-(chord + w['camClearance']):.3f})")
    land_r = w["stopDiameter"] / 2
    print(f"current land R {land_r:.3f}; bore R {bore_r:.3f}; collapsed sleeve R {sleeve_r:.3f}; "
          f"cup end face width at blank OD {blank_r - bore_r:.3f}")

    # ---- current design
    cur = allowed_profile(d)
    print("\nCURRENT: allowed radius at height below seat (z rel. seat = band bottom)")
    for z in (0, -2, -5, zc_min, -10, -15):
        print(f"  z={z:7.2f}  allowed {at(cur, z):.3f}   (closed form no swivel {closed_form_allowed(d, z, zc_min):.3f})")
    print(f"  land R {land_r:.3f} vs allowed at land top (z=0) {at(cur, 0):.3f}: margin {at(cur, 0) - land_r:+.3f}")

    # ---- option A
    print("\nOPTION A: sleeve below the open edge (hob keep-out unchanged, zc_min = %.2f)" % zc_min)
    for s in (0, 2, 3, 5, 7.49, 10, 15):
        a = at(cur, -s)
        print(f"  sleeve starts {s:5.2f} below edge: allowed {a:.3f}, collapsed sleeve R {sleeve_r:.3f} "
              f"margin {a - sleeve_r:.3f}; clamped (R {bore_r:.3f}) margin {a - bore_r:.3f}; "
              f"hob gap {root_r - sleeve_r:.3f} nominal at the tightest point")
    nose = w["cupHeight"] - w["cupEnd"]
    print(f"  nose stop at {nose:.0f} mm above the open edge; stack to band: bottom {nose:.0f}, centre "
          f"{nose - band / 2:.0f}, top {nose - band:.0f} mm")
    lift = 35.0
    print(f"  robot lift now {lift:.0f} mm, mandrel top must be below cup open end: needs protrusion + 5 -> "
          f"{nose + 5:.0f} mm (+{nose + 5 - lift:.0f} mm) or an 18 mm protrusion today")

    # ---- option B
    print("\nOPTION B: sacrificial collar of length Lc below the band (seat = collar bottom)")
    rows = []
    for Lc in (0, 5, 6.5, 8, 10, 12.6, 15.4, 20):
        f = shifted(cur, zc_min, zc_min + Lc)
        rows.append((Lc, zc_min + Lc, [f(z) for z in (0, -2, -5, -8, -12)]))
        print(f"  Lc {Lc:5.1f}: lowest hob centre rel. seat {zc_min + Lc:+6.2f}; allowed R at z=0,-2,-5,-8,-12: "
              + " ".join(f"{f(z):.3f}" for z in (0, -2, -5, -8, -12)))

    def lc_for(ro):
        lo, hi = 0.0, 40.0
        for _ in range(40):
            mid = (lo + hi) / 2
            if shifted(cur, zc_min, zc_min + mid)(0.0) >= ro:
                hi = mid
            else:
                lo = mid
        return hi
    for width in (0.3, 0.6, 1.0, 1.5, blank_r - bore_r):
        ro = bore_r + width
        print(f"  land width {width:.3f} (outer R {ro:.3f}): Lc >= {lc_for(ro):.1f} mm (numeric, with swivel)")
    ls = 16.0
    for Lc in (8, 12.6):
        print(f"  Lc {Lc}: sleeve from the seat {Lc + ls:.1f} mm (band + 8 mm margin); blank height "
              f"{w['cupHeight'] + Lc:.1f}; collar steel {math.pi * (blank_r ** 2 - bore_r ** 2) * Lc * 7.85e-3:.1f} g")

    if len(sys.argv) > 2:
        figure(sys.argv[2], d, cur, rows, zc_min, land_r, sleeve_r, bore_r)


def figure(path, d, cur, rows, zc_min, land_r, sleeve_r, bore_r):
    zs = [-i * 0.25 for i in range(0, 73)]
    sx, sy, x0, y0 = 120.0, 14.0, 60.0, 30.0
    rmin, rmax = 28.5, 33.0

    def P(r, z):
        return f"{x0 + (r - rmin) * sx:.1f},{y0 - z * sy:.1f}"
    curves = [("current (seat = band bottom)", cur, "#b03030", None)]
    for Lc, colour in ((8, "#1d6b3c"), (12.6, "#1d3b8c")):
        f = shifted(cur, zc_min, zc_min + Lc)
        curves.append((f"B, collar {Lc} mm", [(z, f(z)) for z, _ in cur], colour, Lc))
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{x0 + (rmax - rmin) * sx + 220:.0f}" height="{y0 + 18 * sy:.0f}" '
           'font-family="Helvetica,Arial,sans-serif" font-size="11">']
    out.append(f'<text x="{x0}" y="14" font-weight="600">Allowed mandrel/fixture radius below the seat (hob keep-out minus 0.3 mm)</text>')
    for r in (29, 30, 31, 32, 33):
        out.append(f'<line x1="{P(r, 0).split(",")[0]}" y1="{y0}" x2="{P(r, 0).split(",")[0]}" y2="{y0 + 17 * sy:.0f}" stroke="#ddd"/>'
                   f'<text x="{P(r, 0).split(",")[0]}" y="{y0 + 17 * sy + 12:.0f}" text-anchor="middle">R {r}</text>')
    for z in (0, -4, -8, -12, -16):
        out.append(f'<line x1="{x0}" y1="{P(rmin, z).split(",")[1]}" x2="{x0 + (rmax - rmin) * sx:.0f}" y2="{P(rmin, z).split(",")[1]}" stroke="#ddd"/>'
                   f'<text x="{x0 - 6}" y="{float(P(rmin, z).split(",")[1]) + 4:.1f}" text-anchor="end">{z}</text>')
    ly = y0 + 4
    for name, prof, colour, _ in curves:
        pts = [(min(max(r, rmin), rmax), z) for z, r in prof if z >= -16]
        out.append(f'<polyline fill="none" stroke="{colour}" stroke-width="1.6" points="{" ".join(P(r, z) for r, z in pts)}"/>')
        out.append(f'<text x="{x0 + (rmax - rmin) * sx + 10:.0f}" y="{ly}" fill="{colour}">{name}</text>')
        ly += 16
    out.append(f'<line x1="{P(land_r, 0).split(",")[0]}" y1="{P(land_r, 0).split(",")[1]}" x2="{P(land_r, -2).split(",")[0]}" y2="{P(land_r, -2).split(",")[1]}" stroke="#444" stroke-width="4"/>'
               f'<text x="{x0 + (rmax - rmin) * sx + 10:.0f}" y="{ly}" fill="#444">current land R {land_r:.3f}, z -2..0</text>')
    ly += 16
    out.append(f'<line x1="{P(sleeve_r, 0).split(",")[0]}" y1="{P(sleeve_r, 3).split(",")[1]}" x2="{P(sleeve_r, 0).split(",")[0]}" y2="{P(sleeve_r, -3).split(",")[1]}" stroke="#8a6d00" stroke-width="4"/>'
               f'<text x="{x0 + (rmax - rmin) * sx + 10:.0f}" y="{ly}" fill="#8a6d00">option A sleeve R {sleeve_r:.3f}, z -3..</text>')
    out.append(f'<text x="14" y="{y0 + 8 * sy:.0f}" transform="rotate(-90 14 {y0 + 8 * sy:.0f})">z rel. seat (mm)</text></svg>')
    open(path, "w").write("\n".join(out))


if __name__ == "__main__":
    main()
