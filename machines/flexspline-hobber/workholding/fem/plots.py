"""Plots from results/*.json.  Run from repo root after case1-4."""
import json, os, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); R = H + "/results"; P = H + "/plots"; os.makedirs(P, exist_ok=True)
L = lambda n: json.load(open(f"{R}/{n}.json"))
d = L("case1_direct")
fig, ax = plt.subplots(figsize=(7, 4))
for o in d:
    if o["tag"] in ("L4", "L6", "L8", "L12", "L16", "L24", "start2_L16"):
        z, u = o["full_u"]; ax.plot(z, u, label=o["tag"])
ax.axvspan(0, 8, color="0.9", label="tooth band"); ax.set_xlim(0, 30)
ax.set_xlabel("z from open end / seat (mm)"); ax.set_ylabel("radial displacement at root (um per MPa)")
ax.set_title("Case 1: uniform bore pressure over z0..z1"); ax.legend(ncol=2, fontsize=8); fig.tight_layout(); fig.savefig(P + "/case1_uniform_pressure.png", dpi=130)
s = L("case1_sleeve")
fig, axs = plt.subplots(1, 2, figsize=(11, 4), sharey=False)
for ax, P_ in zip(axs, (10.0, 15.0)):
    for o in s:
        if o["P"] == P_ and o["gap_um"] == 5: ax.plot(o["zs"], o["u_um"], label=o["variant"])
    ax.axhline(5, ls="--", c="k", lw=0.7); ax.set_title(f"sleeve oil {P_:g} MPa, radial gap 5 um (dashed)"); ax.set_xlabel("z (mm)")
axs[0].set_ylabel("cup root radial displacement (um)"); axs[0].legend(fontsize=7); fig.tight_layout(); fig.savefig(P + "/case1_sleeve_profiles.png", dpi=130)
c2 = L("case2")
fig, ax = plt.subplots(figsize=(8, 4))
lab = [f"{o['variant']} {o['label']}" for o in c2 if o["label"] != "base"]
val = [np.abs(o["harm_um"][1:4]).max() for o in c2 if o["label"] != "base"]
ax.barh(lab, val); ax.set_xscale("log"); ax.set_xlabel("clamped-rim harmonic amplitude (um)"); ax.set_title("Case 2: wall variation vs OD out-of-round"); fig.tight_layout(); fig.savefig(P + "/case2_harmonics.png", dpi=130)
c3 = L("case3")
fig, ax = plt.subplots(figsize=(9, 5))
lab = [f"{o['kind']} {o['label'][:55]}" for o in c3]; val = [max(abs(o["ur_load_um"]), abs(o["ut_load_um"])) for o in c3]
ax.barh(lab, val); ax.set_xscale("log"); ax.tick_params(labelsize=6); ax.set_xlabel("deflection at load point per 10 N (um), elastic part"); fig.tight_layout(); fig.savefig(P + "/case3_force_deflection.png", dpi=130)
c4 = L("case4")
fig, ax = plt.subplots(figsize=(7, 4))
for o in c4: ax.plot(o["zs"], o["u_um"], label=o["label"])
ax.set_xlabel("z (mm)"); ax.set_ylabel("radial displacement (um per 100 N)"); ax.legend(fontsize=7); fig.tight_layout(); fig.savefig(P + "/case4_axial.png", dpi=130)
