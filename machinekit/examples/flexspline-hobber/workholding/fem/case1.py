"""Case 1: uniform contact pressure (per MPa) and sleeve-driven clamping profile.  Run from repo root."""
from common import *

def direct(args):
    tag, z0, z1, kw = args
    r = c.run_model("c1_" + tag, dict(nth=kw.pop("nth", 12), p_bore=(1.0, z0, z1), **kw))
    s = c.band_stats(r)
    return dict(tag=tag, z0=z0, z1=z1, mean_um=um(s["mean"]), taper_um=um(s["taper"]), u_seat_um=um(s["z0"]), u_bandend_um=um(s["zend"]),
                zs=s["zs"], u_um=s["zmean"] * 1e3, full_u=(r["zs"], r["ur"][0] * 1e3))

cases = [(f"L{L}", 0.0, float(L), {}) for L in (4, 6, 8, 10, 12, 16, 24)]
cases += [("start2_L16", 2.0, 18.0, {}), ("start4_L16", 4.0, 20.0, {}), ("L16_topfixed", 0, 16.0, dict(fix_top_radial=True)),
          ("L16_fine", 0, 16.0, dict(fine=0.5, nr_cup=4)), ("L6_fine", 0, 6.0, dict(fine=0.5, nr_cup=4)),
          ("L16_nth24", 0, 16.0, dict(nth=24))]
out = pmap(direct, cases)
save("case1_direct", out)
for o in out: print(f"{o['tag']:14s} mean {o['mean_um']:.3f} um/MPa taper {o['taper_um']:.3f} seat {o['u_seat_um']:.3f} band-end {o['u_bandend_um']:.3f}")

# sleeve-driven: oil pressure P through a 1.5 mm sleeve with contact; gap 5 um radial (10 um diametral)
def sleeve(args):
    v, P, g = args
    sp = dict(VARIANTS[v]); sp.update(gap=g, oil=P)
    r = c.run_model("c1s_%s_%g_%g" % (v[:2], P, g * 1e3), dict(nth=12, sleeve=sp))
    s = c.band_stats(r)
    contact = [float(u) > g * 0.999 for u in s["zmean"]]
    return dict(variant=v, P=P, gap_um=g * 1e3, mean_um=um(s["mean"]), taper_um=um(s["taper"]), u_seat_um=um(s["z0"]), u_bandend_um=um(s["zend"]),
                zs=s["zs"], u_um=s["zmean"] * 1e3, sleeve_u_um=r["sl_ur"][0] * 1e3, sleeve_z=r["sl_zs"],
                clamped_len_in_band=float(sum(contact) * 0.5))
cases = [(v, P, 0.005) for v in VARIANTS for P in (6.0, 10.0, 15.0)] + [(v, 10.0, 0.010) for v in VARIANTS]
out = pmap(sleeve, cases)
save("case1_sleeve", out)
for o in out: print(f"{o['variant'][:40]:40s} P{o['P']:4.0f} gap{o['gap_um']:4.0f}  mean {o['mean_um']:6.2f} taper {o['taper_um']:6.2f} seat {o['u_seat_um']:6.2f} end {o['u_bandend_um']:6.2f}")
