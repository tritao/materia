"""Case 3: 10 N radial / tangential hob force on the tooth band, clamped by pressurised sleeve vs plain plug.  Run from repo root."""
from common import *
V3 = VARIANTS["V3 free open end, far land 1.5, L16"]; V1 = VARIANTS["V1 lands 1.5/1.5, L16"]
F = 10.0
def spec(kind, **kw):
    d = dict(nth=36, sym="S" if kind == "rad" else "A", force=dict(kind=kind, F=F, thp=5)); d.update(kw); return d

runs = []   # (group, label, kind, spec-kwargs)
for kind in ("rad", "tan"):
    runs.append(("cup alone", "cup, rim stuck on land, no support", kind, spec(kind, land_mode="stuck")))
    runs.append(("sleeve", "V3 sleeve tied (const-pressure oil)", kind, spec(kind, sleeve=dict(V3, gap=0, oil=0, tie=True), land_mode="stuck")))
    runs.append(("sleeve", "V1 sleeve tied (const-pressure oil)", kind, spec(kind, sleeve=dict(V1, gap=0, oil=0, tie=True), land_mode="stuck")))
    runs.append(("sleeve", "V3 sleeve tied, inner face rigid (oil locked)", kind, spec(kind, sleeve=dict(V3, gap=0, oil=0, tie=True, rigid_back=True), land_mode="stuck")))
for cr in (0.0, 2.5e-3, 5e-3):
    runs.append(("plug", f"plug, land slides, radial clearance {cr*1e3:.1f} um (shifted to touch at hob)", "rad",
                 spec("rad", plug=dict(clear_r=cr, shift_x=cr), land_mode="slide", pin_x=False)))
    for kind in ("rad", "tan"):
        runs.append(("plug", f"plug, land stuck, radial clearance {cr*1e3:.1f} um (concentric)", kind,
                     spec(kind, plug=dict(clear_r=cr), land_mode="stuck")))
def go(i):
    g, label, kind, sp = runs[i]
    return c.run_model("c3_%02d" % i, sp)
res = pmap(go, range(len(runs)))
out = []
zs = res[0]["zs"]; k4 = int(np.argmin(abs(zs - 4.0))); sel = zs <= c.BAND + 1e-9
for (g, label, kind, sp), r in zip(runs, res):
    d = dict(group=g, label=label, kind=kind, ur_load_um=um(r["ur"][0, k4]), ut_load_um=um(r["ut"][0, k4]),
             ur_bandmax_um=um(np.abs(r["ur"][:, sel]).max()), ut_bandmax_um=um(np.abs(r["ut"][:, sel]).max()),
             ur_z=r["ur"][0, sel] * 1e3, zs=zs[sel], ur_th=r["ur"][:, k4] * 1e3, ut_th=r["ut"][:, k4] * 1e3, theta=r["th"])
    d["clear_r_um"] = sp.get("plug", {}).get("clear_r", 0) * 1e3 if "plug" in sp else None
    d["rigid_shift_um"] = sp["plug"].get("shift_x", 0) * 1e3 if "plug" in sp and sp.get("land_mode") == "slide" else 0.0
    out.append(d)
    print(f"{kind} {label:70s} ur@load {d['ur_load_um']:8.4f}  ut@load {d['ut_load_um']:8.4f}  max|ur| {d['ur_bandmax_um']:.4f} max|ut| {d['ut_bandmax_um']:.4f}")
save("case3", out)
