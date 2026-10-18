"""Independent triangle-crossing check of the minimum-distance candidate pose."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

root = Path(__file__).resolve().parent / 'build'
right_angle = '--right-angle' in sys.argv
out = root / ('wrist-right-angle-candidate' if right_angle else 'wrist-plug-candidate')
report = json.loads((out / 'layout-report.json').read_text())
manifest = json.loads((root / 'fairino5_v6/manifest.json').read_text())
assert manifest['urdfSha256'] == report['parentUrdfSha256']
cases = [c for c in report['meshCases'] if not right_angle or (c['clockDeg'] == report['preferredClockDeg'] and c['assumedLeadingFaceOffsetMm'] == 0)]
case, pair = min(((c, p) for c in cases for p in c['pairs']), key=lambda x: x[1]['surfaceDistanceMm'])
probe = manifest['probes'][case['sourceProbe']]
assert probe['q'] == case['q']
poses = {p['id']: p['pose'] for p in probe['links']}


def rt(f):
    return Rotation.from_quat([f[k] for k in ('qx', 'qy', 'qz', 'qw')]).as_matrix(), np.array([f[k] for k in ('x', 'y', 'z')])


rw, tw = rt(poses['wrist3_link'])


def robot_mesh(link):
    vertices, indices = [], []
    rotation, translation = rt(poses[link['id']])
    count = 0
    for visual in link['visuals']:
        data = []
        for kind, dtype in [('vertices', '<f8'), ('indices', '<i4')]:
            file = root / 'fairino5_v6' / (visual['stem'] + '.' + kind)
            if not file.exists():
                file = root / 'fairino5_v6/meshes' / (visual['stem'] + '.' + kind)
            raw = file.read_bytes()
            assert hashlib.sha256(raw).hexdigest() == visual['sha256'][kind]
            data.append(np.frombuffer(raw, dtype=dtype).reshape(-1, 3))
        v, i = data
        # Display in wrist3 local millimetres to make the interference easy to inspect.
        vertices.append(((v @ rotation.T + translation - tw) @ rw) * 1000)
        indices.append(i + count); count += len(v)
    return np.concatenate(vertices), np.concatenate(indices)


stem = 'layout-plug' if right_angle else 'plug'
plug = np.fromfile(out / (stem + '.vertices'), dtype='<f8').reshape(-1, 3)
pi = np.fromfile(out / (stem + '.indices'), dtype='<i4').reshape(-1, 3)
external = (plug[pi, 2].min(axis=1) >= (16.002 if right_angle else 15.8) - 1e-7)
rigid = np.ones(len(pi), dtype=bool) if right_angle else (plug[pi, 2].max(axis=1) <= 48.4 + 1e-7)
clock = Rotation.from_euler('z', case['clockDeg'], degrees=True).as_matrix()
plug = plug @ clock.T
plug = np.column_stack((plug[:, 0], -plug[:, 2] - 38 - case['assumedLeadingFaceOffsetMm'], plug[:, 1] + 80))
link = next(l for l in manifest['links'] if l['id'] == pair['link'])
arm, ai = robot_mesh(link)
a = plug[pi[external]]
b = arm[ai]
lo, hi = a.min(axis=(0, 1)), a.max(axis=(0, 1))
b = b[(b.max(axis=1) >= lo).all(axis=1) & (b.min(axis=1) <= hi).all(axis=1)]


def segment_hits(start, end, triangles):
    """Moller-Trumbore with a bounded segment and strictly interior triangle hit."""
    direction = end - start
    e1 = triangles[:, 1] - triangles[:, 0]
    e2 = triangles[:, 2] - triangles[:, 0]
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    det = np.einsum('ij,ij->i', e1, h)
    valid = np.abs(det) > 1e-10
    inv = np.zeros_like(det); inv[valid] = 1 / det[valid]
    s = start - triangles[:, 0]
    u = inv * np.einsum('ij,ij->i', s, h)
    q = np.cross(s, e1)
    v = inv * (q @ direction)
    t = inv * np.einsum('ij,ij->i', e2, q)
    hit = np.flatnonzero(valid & (u > 1e-7) & (v > 1e-7) & (u + v < 1 - 1e-7) & (t > 1e-7) & (t < 1 - 1e-7))
    return None if not len(hit) else start + t[hit[0]] * direction


# Analytic calibration: a segment through an interior point, and a translated miss.
tri = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
assert np.allclose(segment_hits(np.array([.2, .2, -1]), np.array([.2, .2, 1]), tri), [.2, .2, 0])
assert segment_hits(np.array([2., 2., -1]), np.array([2., 2., 1]), tri) is None
def find_crossing(triangles, obstacles):
    for triangle in triangles:
        nearby = obstacles[(obstacles.max(axis=1) >= triangle.min(axis=0)).all(axis=1) & (obstacles.min(axis=1) <= triangle.max(axis=0)).all(axis=1)]
        for i in range(3):
            point = segment_hits(triangle[i], triangle[(i + 1) % 3], nearby)
            if point is not None:
                return point
    return None


rigid_hit = find_crossing(plug[pi[external & rigid]], b)
hit = rigid_hit if rigid_hit is not None else find_crossing(a, b)
if hit is None and pair['surfaceDistanceMm'] <= .001:
    raise RuntimeError('Native zero was not independently resolved as a strict triangle crossing')
if hit is not None and pair['surfaceDistanceMm'] > .001:
    raise RuntimeError('Independent triangle crossing contradicts positive native surface distance')

from matplotlib.collections import PolyCollection
fig, ax = plt.subplots(figsize=(9, 5.5), facecolor='#fafbfc')
for v, indices, colour, alpha in [(arm, ai, '#d49575', .5), (*robot_mesh(next(l for l in manifest['links'] if l['id'] == 'wrist3_link')), '#a2abb1', 1), (plug, pi, '#268ac0', 1)]:
    triangles = v[indices][:, :, [1, 2]]
    ax.add_collection(PolyCollection(triangles, facecolors=colour, edgecolors='none', alpha=alpha, rasterized=True))
if hit is not None:
    ax.scatter(hit[1], hit[2], s=100, c='red', zorder=10)
    ax.annotate('Verified triangle crossing', (hit[1], hit[2]), xytext=(-90, 107), arrowprops={'arrowstyle': '->', 'color': 'red'}, color='#9b1919')
else:
    ax.text(-100, 114, f'Native minimum surface gap: {pair["surfaceDistanceMm"]:.2f} mm\nNo strict triangle crossing found in independent screen', fontsize=9)
ax.set(xlim=(-112, 48), ylim=(38, 126), xlabel='Wrist Y (mm)', ylabel='Wrist Z (mm)')
ax.set_aspect('equal'); ax.grid(alpha=.2)
region = ('plug body' if rigid_hit is not None else 'supplier cable stub') if hit is not None else None
fig.suptitle('Right-angle candidate: limiting nominal pose' if right_angle else 'Folded pose: candidate ' + region + ' crosses forearm mesh', fontsize=13)
fig.text(.5, .045, f'Projected supplier triangles; source FK probe {case["sourceProbe"]}; assumed offset {case["assumedLeadingFaceOffsetMm"]} mm, clock {case["clockDeg"]}°.', ha='center', fontsize=9)
fig.tight_layout(rect=(0, .08, 1, .94))
fig.savefig(out / ('limiting-nominal-pose.png' if right_angle else 'folded-pose-interference.png'), dpi=160)
review = {'sourceProbe': case['sourceProbe'], 'q': case['q'], 'link': pair['link'], 'assumedLeadingFaceOffsetMm': case['assumedLeadingFaceOffsetMm'], 'clockDeg': case['clockDeg'], 'nativeSurfaceDistanceMm': pair['surfaceDistanceMm'], 'crossingFound': hit is not None, 'crossingPointWristMm': None if hit is None else hit.tolist(), 'crossingRegion': region, 'rigidBodyTriangleCrossingFound': rigid_hit is not None, 'method': 'Strict interior segment/triangle intersection on supplier plug exterior triangles and source ROS visual triangles; analytic hit/miss calibration passed', 'conclusion': ('Triangle surfaces cross.' if hit is not None else 'No strict triangle crossing found in this screen; native surface distance is positive. This is not a volume containment test.') + ' No signed solid penetration depth is inferred; this does not qualify mating or certify a tending motion.'}
(out / 'independent-mesh-review.json').write_text(json.dumps(review, indent=2) + '\n')
print(json.dumps(review, indent=2))
