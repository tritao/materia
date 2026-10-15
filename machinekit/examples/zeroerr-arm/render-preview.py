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

revised = "--revised" in sys.argv
root = Path(__file__).resolve().parent / "build"
if revised: root /= "revised"
stem = "zeroerr5-wrist80" if revised else "zeroerr5"
parts = json.loads((root / (stem+".mesh.json")).read_text())
triangles, colors, points = [], [], []
light = np.array([-0.3, -0.6, 1])
light /= np.linalg.norm(light)
for part in parts:
    vertices = np.array(part["vertices"]).reshape(-1, 3)
    faces = np.array(part["indices"]).reshape(-1, 3)
    tri = vertices[faces]
    points.append(vertices)
    color = "#e6a24c" if part["id"] in ["upperArm", "forearm", "elbowOutputAdapter",
                                             "elbowHousingAdapter", "toolFlange"] else "#718897"
    if "-detail" in part["id"]:
        color = "#566671"
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    shade = 0.48 + 0.52 * np.abs(normals @ light)
    triangles.append(tri)
    colors.append(np.array(matplotlib.colors.to_rgb(color))[None, :] * shade[:, None])
vertices = np.concatenate(points)
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
fig.suptitle("ZeroErr · 80T wrist engineering revision" if revised else "ZeroErr 5 kg reference · articulated CAD", fontsize=20, y=0.96)
fig.text(0.5, 0.91, "T-shaped joint modules · straight hollow links · supplier mounting geometry",
         ha="center", fontsize=12, color="#44515c")
fig.text(0.5, 0.035, "Mechanical review model · motor configuration and load capability remain to be validated",
         ha="center", fontsize=10, color="#44515c")
fig.subplots_adjust(left=0, right=1, bottom=0.04, top=0.91)
fig.savefig(root / (stem+"-preview.png"), dpi=160)
print("Saved", root / (stem+"-preview.png"))
