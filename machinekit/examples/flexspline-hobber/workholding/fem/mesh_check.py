"""Refinement check for the 3D force cases (linear, tie/stuck).  Run from repo root."""
from common import *
V3 = VARIANTS["V3 free open end, far land 1.5, L16"]
out = []
for tag, kw in (("base nth36 fine1 nr2", {}), ("refined nth72 fine0.5 nr4", dict(nth=72, fine=0.5, nr_cup=4))):
    for name, extra in (("cup alone stuck", dict(land_mode="stuck")),
                        ("V3 sleeve tied", dict(land_mode="stuck", sleeve=dict(V3, gap=0, oil=0, tie=True)))):
        sp = dict(nth=36, sym="S", force=dict(kind="rad", F=10, thp=5)); sp.update(extra); sp.update(kw)
        # sleeve block has 3 radial elements; refined axial grid handled by fine
        r = c.run_model("mc_%s_%s" % (tag[:4], name[:4]), sp)
        k = int(np.argmin(abs(r["zs"] - 4.0)))
        out.append(dict(mesh=tag, model=name, ur_load_um=um(r["ur"][0, k])))
        print(tag, name, um(r["ur"][0, k]))
save("mesh_check", out)
