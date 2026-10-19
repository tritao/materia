#!/usr/bin/env python3
"""Render exported mold CAD triangles; the second image hides two walls for inspection."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def refine(triangles, limit=35.0):
    """Split long planar mesh edges so painter sorting is less sensitive to big faces."""
    result = []
    pending = triangles
    while len(pending):
        lengths = np.sum((pending[:, [1, 2, 0]] - pending) ** 2, axis=2)
        edges = lengths.argmax(axis=1)
        large = lengths.max(axis=1) > limit * limit
        result.append(pending[~large])
        if not large.any():
            break
        selected, edge = pending[large], edges[large]
        row = np.arange(len(selected))
        a, b, c = selected[row, edge], selected[row, (edge + 1) % 3], selected[row, (edge + 2) % 3]
        midpoint = (a + b) / 2
        pending = np.concatenate((np.stack((a, midpoint, c), axis=1), np.stack((midpoint, b, c), axis=1)))
    return np.concatenate(result)

root = Path(__file__).resolve().parent / "build/tooling/base"
light = np.array([.3, -.5, .8]); light /= np.linalg.norm(light)
for cutaway in (False, True):
    fig = plt.figure(figsize=(15, 8))
    for index, name in enumerate(("mold", "mold-exploded"), 1):
        ax = fig.add_subplot(1, 2, index, projection="3d")
        points, all_triangles, all_colors = [], [], []
        for row in json.loads((root / f"{name}-meshes.json").read_text()):
            member = row["id"].removeprefix(name + "-")
            if cutaway and name == "mold" and member in ("south", "west"):
                continue
            vertices = np.fromfile(root / (row["id"] + ".vertices"), dtype="<f8").reshape(-1, 3)
            faces = np.fromfile(root / (row["id"] + ".indices"), dtype="<i4").reshape(-1, 3)
            frame = row["pose"]
            q = np.array([frame[k] for k in ("qx", "qy", "qz")])
            vertices += 2 * np.cross(q, np.cross(q, vertices) + frame["qw"] * vertices)
            vertices += np.array([frame[k] for k in "xyz"])
            triangles = refine(vertices[faces]); points.append(vertices)
            normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
            colors = np.asarray(row["color"])[None, :] * (.5 + .5 * np.abs(normals @ light))[:, None]
            all_triangles.append(triangles); all_colors.append(colors)
            if name == "mold-exploded" and member in ("bed", "north", "east", "south", "west"):
                ax.text(*vertices.mean(axis=0), member, fontsize=7)
        ax.add_collection3d(Poly3DCollection(np.concatenate(all_triangles), facecolors=np.concatenate(all_colors), edgecolors="none", rasterized=True))
        points = np.concatenate(points); low, high = points.min(axis=0), points.max(axis=0)
        for axis, lo, hi in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), low, high):
            axis(lo - 15, hi + 15)
        ax.set_box_aspect(high - low + 30); ax.view_init(elev=32, azim=-55)
        ax.set_title("Assembled" + (" — south/west walls hidden" if cutaway and index == 1 else "") if index == 1 else "Exploded review")
        ax.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
    fig.suptitle("Bench-mill base mold — six EPS cores, detail formers and nominal bed-referenced insert supports")
    fig.text(.5, .02, "REVIEW ONLY: fixture attachments, replication/grout allowances, anchorage, compaction, loads and handling remain pending.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .05, 1, .95))
    target = root / ("mold-cutaway.png" if cutaway else "mold-review.png")
    fig.savefig(target, dpi=150); plt.close(fig); print(target)
