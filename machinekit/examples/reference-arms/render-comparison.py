#!/usr/bin/env python3
"""Render the saved combined scene's checked mesh streams at a common scale."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parent / 'build/comparison'
IDS = ('zeroerr', 'ur5e', 'fairino5_v6')
LABELS = ('Our ZeroErr prototype', 'UR5e', 'FAIRINO FR5')


def transform(vertices, frame):
    q = np.array([frame[k] for k in ('qx', 'qy', 'qz')])
    return vertices + 2*np.cross(q, np.cross(q, vertices)+frame['qw']*vertices) + np.array([frame[k] for k in 'xyz'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mounting', choices=('floor', 'wall', 'ceiling'), default='floor')
    parser.add_argument('--closeup', action='store_true')
    args = parser.parse_args()
    directory = ROOT / args.mounting
    meshes = json.loads((directory/'meshes.json').read_text())
    groups = []
    light = np.array([-.3, -.6, 1.]); light /= np.linalg.norm(light)
    for index, model in enumerate(IDS):
        triangles, colors = [], []
        for item in meshes:
            if not item['id'].startswith(model+'/'):
                continue
            vertices = np.fromfile(directory/(item['stem']+'.vertices'), dtype='<f8').reshape(-1, 3)
            faces = np.fromfile(directory/(item['stem']+'.indices'), dtype='<i4').reshape(-1, 3)
            world = transform(vertices, item['pose'])
            world[:, 0] -= (index-1)*1.2
            tri = world[faces]
            normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
            normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-20)
            shade = .45 + .55*np.abs(normal@light)
            triangles.append(tri)
            colors.append(np.array(item['color'])[None, :]*shade[:, None])
            if item['id'].endswith('/comparison-tool'):
                tool_center = world.mean(axis=0)
        groups.append((np.concatenate(triangles), np.concatenate(colors), tool_center))
    points = np.concatenate([g[0].reshape(-1, 3) for g in groups])
    low, high = points.min(axis=0), points.max(axis=0)
    center = (low+high)/2
    span = max(high-low)*.56
    if args.closeup:
        center = np.mean([g[2] for g in groups], axis=0)
        span = .23
    fig = plt.figure(figsize=(18, 8), facecolor='#edf0f3')
    for i, (triangles, colors, tool_center) in enumerate(groups):
        ax = fig.add_subplot(1, 3, i+1, projection='3d')
        ax.set_facecolor('#edf0f3')
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolors='none'))
        ax.set(xlim=(center[0]-span, center[0]+span), ylim=(center[1]-span, center[1]+span),
               zlim=(center[2]-span, center[2]+span))
        ax.set_box_aspect((1, 1, 1)); ax.set_proj_type('ortho'); ax.view_init(elev=20, azim=-65)
        ax.set_axis_off(); ax.set_title(LABELS[i], fontsize=17)
    fig.suptitle('Common physical tool and TCP task · '+args.mounting+' mounting', fontsize=22)
    fig.text(.5, .045, 'Shared scale · 40 × 40 × 150 mm tool envelope · base-relative task · display spacing removed in panels',
             ha='center', fontsize=10, color='#44515c')
    fig.text(.5, .018, 'Prototype geometry and simulated kinematics; mounting hardware, cell registration and dynamics remain unqualified',
             ha='center', fontsize=10, color='#44515c')
    fig.subplots_adjust(left=0, right=1, bottom=.06, top=.88, wspace=0)
    output = ROOT/(args.mounting+('-closeup.png' if args.closeup else '-preview.png'))
    fig.savefig(output, dpi=150)
    print(output)


if __name__ == '__main__':
    main()
