#!/usr/bin/env python3
"""Render the exported CAD triangles in assembled and exploded poses (mm)."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

root = Path(__file__).resolve().parent / "build/review"
fig = plt.figure(figsize=(14, 7))
for index, name in enumerate(("assembled", "exploded"), 1):
    ax = fig.add_subplot(1, 2, index, projection="3d")
    points = []
    for row in json.loads((root / f"{name}-meshes.json").read_text()):
        vertices = np.fromfile(root / (row["id"] + ".vertices"), dtype="<f8").reshape(-1, 3)
        faces = np.fromfile(root / (row["id"] + ".indices"), dtype="<i4").reshape(-1, 3)
        frame = row["pose"]
        q = np.array([frame[k] for k in ("qx", "qy", "qz")])
        vertices = vertices + 2 * np.cross(q, np.cross(q, vertices) + frame["qw"] * vertices)
        vertices += np.array([frame[k] for k in "xyz"])
        points.append(vertices)
        triangles = vertices[faces]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        light = np.array([.3, -.5, .8]); light /= np.linalg.norm(light)
        colors = np.asarray(row["color"])[None, :] * (.5 + .5 * np.abs(normals @ light))[:, None]
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolors="none", rasterized=True))
        center = vertices.mean(axis=0)
        ax.text(*center, row["id"].removeprefix(name + "-"), fontsize=7)
    points = np.concatenate(points)
    low, high = points.min(axis=0), points.max(axis=0)
    for axis, lo, hi in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), low, high):
        axis(lo - 10, hi + 10)
    ax.set_box_aspect(high - low + 20)
    ax.view_init(elev=32, azim=-55)
    ax.set_title(name.capitalize())
    ax.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
fig.suptitle("Single-pocket mold — CAD geometry review")
fig.text(.5, .02, "Physical restraint, datum accuracy, sealing, loads and handling remain pending.", ha="center")
fig.tight_layout(rect=(0, .05, 1, .95))
fig.savefig(root / "mold-review.png", dpi=160)
print(root / "mold-review.png")
