"""Case 4: 100 N axial pull-down on the 0.3 mm stop land.  Run from repo root."""
from common import *
runs = [("free cup, rim slides on land", dict(land_mode="slide", pin_x=True)),
        ("free cup, rim stuck on land", dict(land_mode="stuck")),
        ("zero-clearance plug backing, rim slides", dict(land_mode="slide", pin_x=True, plug=dict(clear_r=0.0))),
        ("free cup, rim slides, sleeve-tied V1 (clamped)", dict(land_mode="slide", pin_x=True,
              sleeve=dict(VARIANTS["V1 lands 1.5/1.5, L16"], gap=0, oil=0, tie=True)))]
def go(i):
    lab, kw = runs[i]
    return c.run_model("c4_%d" % i, dict(nth=12, land_contact=True, axial=100.0, **kw))
res = pmap(go, range(len(runs)))
out = []
for (lab, kw), r in zip(runs, res):
    s = c.band_stats(r)
    sel = r["zs"] <= c.BAND + 1e-9
    d = dict(label=lab, mean_um=um(s["mean"]), taper_um=um(s["taper"]), seat_um=um(s["z0"]), bandend_um=um(s["zend"]),
             circ_pp_um=um(s["circ_pp"]), uz_top_um=um(r["uz"][0, -1]), zs=s["zs"], u_um=s["zmean"] * 1e3)
    out.append(d)
    print(f"{lab:50s} mean {d['mean_um']:.4f} taper {d['taper_um']:.4f} seat {d['seat_um']:.4f} bandend {d['bandend_um']:.4f} uz_top {d['uz_top_um']:.3f}")
save("case4", out)
