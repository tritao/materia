#!/usr/bin/env python3
"""Render prepared reference geometry at source-URDF probe poses, at a shared scale."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parent


def transform(vertices, frame):
    x, y, z, w = [frame[k] for k in ("qx", "qy", "qz", "qw")]
    q = np.array([x, y, z])
    return vertices + 2 * np.cross(q, np.cross(q, vertices) + w * vertices) + np.array([frame[k] for k in "xyz"])


def review_poses(manifest):
    q = [0, -1, 1.3, -1.5, -1, 0] if manifest["family"] == "ur" else [np.pi, -1, 1.3, -1.5, -1, 0]
    poses = {manifest["root"]: np.eye(4)}
    index = 0
    for joint in manifest["joints"]:
        f = joint["origin"]
        # Transform the basis to build the homogeneous matrix independently.
        matrix = np.eye(4)
        matrix[:3, 3] = [f[k] for k in "xyz"]
        matrix[:3, :3] = (transform(np.eye(3), f) - matrix[:3, 3]).T
        motion = np.eye(4)
        if joint["type"] != "fixed":
            value = q[index]; index += 1
            x, y, z = joint["axis"]
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            motion[:3, :3] = np.eye(3) + np.sin(value)*skew + (1-np.cos(value))*(skew @ skew)
        poses[joint["child"]] = poses[joint["parent"]] @ matrix @ motion
    return poses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="*", default=["ur5e", "fairino5_v6"])
    parser.add_argument("--probe", type=int)
    args = parser.parse_args()
    all_models = []
    light = np.array([-.3, -.6, 1]); light /= np.linalg.norm(light)
    for name in args.models:
        directory = ROOT / "build" / name
        manifest = json.loads((directory / "manifest.json").read_text())
        poses = review_poses(manifest) if args.probe is None else {
            row["id"]: row["pose"] for row in manifest["probes"][args.probe]["links"]}
        triangles, colors = [], []
        for link in manifest["links"]:
            for mesh in link["visuals"]:
                v = np.fromfile(directory / (mesh["stem"] + ".vertices"), dtype="<f8").reshape(-1, 3)
                f = np.fromfile(directory / (mesh["stem"] + ".indices"), dtype="<i4").reshape(-1, 3)
                pose = poses[link["id"]]
                world = v @ pose[:3, :3].T + pose[:3, 3] if args.probe is None else transform(v, pose)
                tri = world[f]
                normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
                normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
                shade = .45 + .55 * np.abs(normals @ light)
                triangles.append(tri)
                colors.append(np.array(mesh["color"][:3])[None, :] * shade[:, None])
        all_models.append((name, np.concatenate(triangles), np.concatenate(colors)))
    points = np.concatenate([tri.reshape(-1, 3) for _, tri, _ in all_models])
    low, high = points.min(0), points.max(0)
    midpoint = (low + high) / 2; span = max(high - low) * .58
    fig = plt.figure(figsize=(7 * len(all_models), 8), facecolor="#edf0f3")
    for i, (name, triangles, colors) in enumerate(all_models):
        ax = fig.add_subplot(1, len(all_models), i + 1, projection="3d")
        ax.set_facecolor("#edf0f3")
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolors="none"))
        ax.set(xlim=(midpoint[0]-span, midpoint[0]+span), ylim=(midpoint[1]-span, midpoint[1]+span),
               zlim=(midpoint[2]-span, midpoint[2]+span))
        ax.set_box_aspect((1, 1, 1)); ax.set_proj_type("ortho"); ax.view_init(elev=20, azim=-65)
        ax.set_axis_off(); ax.set_title(name, fontsize=18)
    fig.suptitle("Official robot descriptions · shared scale", fontsize=21)
    label = "Illustrative configurations; joint coordinates differ" if args.probe is None else f"Independent source-FK pose {args.probe}"
    fig.text(.5, .035, f"{label} · shared scale · reference geometry, not a dynamics qualification",
             ha="center", color="#44515c", fontsize=10)
    fig.subplots_adjust(left=0, right=1, bottom=.05, top=.9, wspace=0)
    path = ROOT / "build/reference-preview.png"
    fig.savefig(path, dpi=150)
    print(path)


if __name__ == "__main__":
    main()
