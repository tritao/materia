"""Render actual supplier triangles for the explicitly assumed nominal candidate placement."""
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

root = Path(__file__).resolve().parent / 'build'
right_angle = '--right-angle' in sys.argv
out = root / ('wrist-right-angle-candidate' if right_angle else 'wrist-plug-candidate')
spec = json.loads((root.parent / ('wrist-right-angle-candidate.json' if right_angle else 'wrist-plug-candidate.json')).read_text())
geometry = json.loads((out / 'supplier-geometry.json').read_text())
assert geometry['sha256'] == spec['files'][0]['sha256']
assert (out / ('layout-plug.vertices' if right_angle else 'fr5-candidate-offset-0.mtrg')).exists()


def mesh(directory, name):
    vertices = np.fromfile(directory / (name + '.vertices'), dtype='<f8').reshape(-1, 3)
    indices = np.fromfile(directory / (name + '.indices'), dtype='<i4').reshape(-1, 3)
    return vertices, indices


wrist, wi = mesh(root / 'wrist-geometry-inspection', 'step-solid-36')
wrist += [820, -50, -102]
plug, pi = mesh(out, 'layout-plug' if right_angle else 'plug')
# Apply the selected study clock, then the wrist observation frame Rx(90 degrees).
if right_angle:
    angle = np.deg2rad(spec['preferredClockDeg'])
    clock = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    plug = plug @ clock.T
plug = np.column_stack((plug[:, 0], -plug[:, 2] - 38, plug[:, 1] + 80))
fig = plt.figure(figsize=(11, 6), facecolor='#fafbfc')
for index, (elev, azim) in enumerate([(25, -145), (80, -90)]):
    ax = fig.add_subplot(1, 2, index + 1, projection='3d')
    for v, triangles, colour in [(wrist, wi, '#9da6ad'), (plug, pi, '#268ac0')]:
        tri = v[triangles]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        light = np.array([-.5, -.5, 1.0]); light /= np.linalg.norm(light)
        rgb = np.array(matplotlib.colors.to_rgb(colour))
        shade = .65 + .35 * np.abs(normals @ light)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=shade[:, None] * rgb, linewidths=0, rasterized=True))
    ax.set(xlim=(-46, 46), ylim=(-98, 46), zlim=(45, 130 if right_angle else 108), xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
    ax.set_box_aspect((92, 144, 85 if right_angle else 63)); ax.view_init(elev=elev, azim=azim)
    if index == 1:
        ax.set_zticks([]); ax.set_zlabel(''); ax.set_proj_type('ortho')
    ax.set_title('Supplier geometry — illustrative alignment', fontsize=10)
fig.suptitle('FR5 wrist + ' + ('right-angle 889D-E5AC-* layout' if right_angle else 'Binder 77 3429 0000 20005-0500 candidate'), fontsize=14)
fig.text(.5, .055, ('Blue: supplier right-angle connector with short cable stub. Exit points back along the robot arm.' if right_angle else 'Blue: imported supplier STEP. Plug leading face aligned to observed socket insert (assumed zero offset).'), ha='center', fontsize=9)
fig.text(.5, .025, 'Mating fit / key clocking unqualified. No flexible cable, hub, grippers or motion certification.', ha='center', fontsize=9)
fig.subplots_adjust(left=.01, right=.99, bottom=.14, top=.88, wspace=.04)
target = out / ('fr5-right-angle-candidate.png' if right_angle else 'fr5-plug-candidate.png')
fig.savefig(target, dpi=160)
print(target)
