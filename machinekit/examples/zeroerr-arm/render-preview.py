#!/usr/bin/env python3
"""Optional static review image from exported CAD meshes (numpy + matplotlib)."""
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

straight = "--straight" in sys.argv
revised = "--revised" in sys.argv or straight
closeup = "--closeup" in sys.argv
root = Path(__file__).resolve().parent / "build"
if revised: root /= "straight" if straight else "revised"
stem = "zeroerr5-straight80" if straight else "zeroerr5-wrist80" if revised else "zeroerr5"
parts = json.loads((root / (stem+".mesh.json")).read_text())
triangles, colors, points, focus = [], [], [], []
light = np.array([-0.3, -0.6, 1])
light /= np.linalg.norm(light)
for part in parts:
    vertices = np.array(part["vertices"]).reshape(-1, 3)
    faces = np.array(part["indices"]).reshape(-1, 3)
    tri = vertices[faces]
    points.append(vertices)
    if part["id"].startswith("forearm"): focus.append(vertices)
    color = "#e6a24c" if part["id"] in ["upperArm", "forearm", "elbowOutputAdapter",
                                             "elbowHousingAdapter", "toolFlange"] else "#718897"
    if "-detail" in part["id"]:
        color = "#566671"
    if part["id"] in ["forearm-proximal", "forearm-distal"]: color = "#e6a24c"
    if part["id"].endswith("-cover"): color = "#445a6a"
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    shade = 0.48 + 0.52 * np.abs(normals @ light)
    triangles.append(tri)
    colors.append(np.array(matplotlib.colors.to_rgb(color))[None, :] * shade[:, None])
vertices = np.concatenate(focus if closeup else points)
low, high = vertices.min(0), vertices.max(0)
mid, span = (low + high) / 2, max(high - low) * 0.56
fig = plt.figure(figsize=(11, 10), facecolor="#edf0f3")
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("#edf0f3")
ax.add_collection3d(Poly3DCollection(np.concatenate(triangles), facecolors=np.concatenate(colors), edgecolors="none"))
ax.set(xlim=(mid[0]-span, mid[0]+span), ylim=(mid[1]-span, mid[1]+span), zlim=(mid[2]-span, mid[2]+span))
ax.set_box_aspect((1, 1, 1))
ax.set_proj_type("ortho")
ax.view_init(elev=20, azim=-60)
ax.set_axis_off()
fig.suptitle("ZeroErr · straight forearm and adapters" if straight else "ZeroErr · 80T wrist engineering revision" if revised else "ZeroErr 5 kg reference · articulated CAD", fontsize=20, y=0.96)
fig.text(0.5, 0.91, "70 × 4 mm straight tube · retained supplier ends · separate service covers" if straight else "T-shaped joint modules · hollow links · supplier mounting geometry",
         ha="center", fontsize=12, color="#44515c")
fig.text(0.5, 0.035, "Mechanical review model · motor configuration and load capability remain to be validated",
         ha="center", fontsize=10, color="#44515c")
fig.subplots_adjust(left=0, right=1, bottom=0.04, top=0.91)
path = root / (stem+("-closeup.png" if closeup else "-preview.png"))
fig.savefig(path, dpi=160)
print("Saved", path)
