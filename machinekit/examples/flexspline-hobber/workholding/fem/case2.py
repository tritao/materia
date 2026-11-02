"""Case 2: non-uniform sleeve wall / out-of-round sleeve OD -> clamped rim shape (harmonics).  Run from repo root."""
from common import *
V3 = VARIANTS["V3 free open end, far land 1.5, L16"]; V1 = VARIANTS["V1 lands 1.5/1.5, L16"]
runs = [("V3", V3, 6.0, "base", {})]
for n in (1, 2):
    for a in (2, 5): runs.append(("V3", V3, 6.0, f"wall n={n} a={a}um", dict(wall_n=n, wall_a=a * 1e-3)))
for n in (1, 2, 3): runs.append(("V3", V3, 6.0, f"OD n={n} b=1um", dict(od_n=n, od_b=1e-3)))
runs.append(("V1", V1, 15.0, "base", {}))
for n in (1, 2): runs.append(("V1", V1, 15.0, f"wall n={n} a=5um", dict(wall_n=n, wall_a=5e-3)))
runs.append(("V1", V1, 15.0, "OD n=2 b=1um", dict(od_n=2, od_b=1e-3)))

def go(a):
    tag, v, P, label, kw = a
    sp = dict(v); sp.update(gap=0.005, oil=P, **kw)
    r = c.run_model("c2_%s_%s" % (tag, label.replace(" ", "_").replace("=", "")), dict(nth=36, sleeve=sp))
    return tag, P, label, r
res = pmap(go, runs)
base = {tag: r for tag, P, l, r in res if l == "base"}
out = []
for tag, P, label, r in res:
    b = base[tag]
    sel = r["zs"] <= c.BAND + 1e-9
    d = (r["ur"] - b["ur"])[:, sel]                     # delta radial displacement, theta x z
    bandmean = d.mean(1)
    h = c.harmonics(r["th"], bandmean, 4)
    hz = [c.harmonics(r["th"], d[:, k], 3) for k in range(d.shape[1])]
    out.append(dict(variant=tag, P=P, label=label, mean_um=um(r["ur"][:, sel].mean()), harm_um=h * 1e3,
                    peak_um=um(np.abs(d).max()), pp_um=um((d.max(0) - d.min(0)).max()),
                    harm_n1_over_z_um=[um(x[1]) for x in hz], harm_n2_over_z_um=[um(x[2]) for x in hz],
                    harm_n3_over_z_um=[um(x[3]) for x in hz], zs=r["zs"][sel], theta=r["th"], d_um=d * 1e3))
    print(f"{tag} {label:20s} base-mean {um(r['ur'][:, sel].mean()):6.2f}  harmonics n0..3 (um) {np.round(h[:4]*1e3,4)}  peak delta {um(np.abs(d).max()):.4f}")
save("case2", out)
