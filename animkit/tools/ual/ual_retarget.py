"""Retargeting of Universal Animation Library clips from Godot's humanoid skeleton to the library's glTF rig.

The library ships its animations for Godot, on a skeleton whose bones are named for Godot's humanoid profile and
whose rest pose is a T-pose. The library's own glTF rig (the character in ual-standard.glb) names its bones
"DEF-..." and rests in a different pose. For a bone b with parent p, the two agree on the animated world pose
up to a constant rotation D per bone, which gives, for the local rotations,

    local_gltf(b) = D(p)^-1 * local_godot(b) * D(b)

Each D is solved top-down from one clip that both forms carry (PickUp_Table, which the free glTF includes), and the
fit is checked against that clip's own keys.
"""
import json
import numpy as np


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qinv(a):
    return np.array([-a[0], -a[1], -a[2], a[3]]) / np.dot(a, a)


def qnorm(a):
    return a / np.linalg.norm(a)


def rotate(q, v):
    u = np.array(q[:3])
    return v + 2 * q[3] * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


def slerp(a, b, t):
    d = np.dot(a, b)
    if d < 0:
        b = -b
        d = -d
    if d > 0.9995:
        return qnorm(a + (b - a) * t)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * a + np.sin(t * th) * b) / np.sin(th)


def sample(keys, t):
    """keys: rows of [time, transition, x, y, z, w]."""
    ts = keys[:, 0]
    if t <= ts[0]:
        return keys[0, 2:6]
    if t >= ts[-1]:
        return keys[-1, 2:6]
    i = np.searchsorted(ts, t) - 1
    return slerp(keys[i, 2:6], keys[i + 1, 2:6], (t - ts[i]) / (ts[i + 1] - ts[i]))


FIXED = {'Root': 'root', 'Hips': 'DEF-hips', 'Spine': 'DEF-spine.001', 'Chest': 'DEF-spine.002',
         'UpperChest': 'DEF-spine.003', 'Neck': 'DEF-neck', 'Head': 'DEF-head'}
LIMBS = {'Shoulder': 'shoulder', 'UpperArm': 'upper_arm', 'LowerArm': 'forearm', 'Hand': 'hand', 'UpperLeg': 'thigh',
         'LowerLeg': 'shin', 'Foot': 'foot', 'Toes': 'toe'}
FINGERS = (('Index', 'f_index'), ('Middle', 'f_middle'), ('Little', 'f_pinky'), ('Ring', 'f_ring'))


def godot_to_def(path):
    """The library glTF bone for a Godot track path such as '%GeneralSkeleton:LeftUpperArm', or None."""
    name = path.split(':')[-1]
    if name in FIXED:
        return FIXED[name]
    side = 'L' if name.startswith('Left') else 'R' if name.startswith('Right') else None
    if side is None:
        return None
    part = name[4:] if side == 'L' else name[5:]
    if part in LIMBS:
        return 'DEF-%s.%s' % (LIMBS[part], side)
    for godot, gltf in FINGERS:
        if part.startswith(godot):
            segment = {'Proximal': '01', 'Intermediate': '02', 'Distal': '03'}[part[len(godot):]]
            return 'DEF-%s.%s.%s' % (gltf, segment, side)
    if part.startswith('Thumb'):
        return 'DEF-thumb.%s.%s' % ({'Metacarpal': '01', 'Proximal': '02', 'Distal': '03'}[part[5:]], side)
    return None


class Gltf:
    """The library glTF: its nodes, parent map, and accessor reads."""

    def __init__(self, gltf_path, bin_path):
        self.json = json.load(open(gltf_path))
        self.bin = open(bin_path, 'rb').read()
        self.nodes = self.json['nodes']
        self.parent = {}
        for node in self.nodes:
            for child in node.get('children', []):
                self.parent[self.nodes[child]['name']] = node['name']
        self.index = {n['name']: i for i, n in enumerate(self.nodes)}

    def accessor(self, k):
        a = self.json['accessors'][k]
        view = self.json['bufferViews'][a['bufferView']]
        width = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[a['type']]
        return np.frombuffer(self.bin, '<f4', a['count'] * width,
                             view.get('byteOffset', 0) + a.get('byteOffset', 0)).reshape(a['count'], width)

    def rotations(self, clip):
        animation = [x for x in self.json['animations'] if x['name'] == clip][0]
        out = {}
        for channel in animation['channels']:
            if channel['target']['path'] != 'rotation':
                continue
            sampler = animation['samplers'][channel['sampler']]
            out[self.nodes[channel['target']['node']]['name']] = (self.accessor(sampler['input']).ravel(),
                                                                  self.accessor(sampler['output']))
        return out


def godot_rotations(animation):
    out = {}
    i = 0
    while 'tracks/%d/type' % i in animation:
        if animation['tracks/%d/type' % i] == 'rotation_3d':
            bone = godot_to_def(animation['tracks/%d/path' % i])
            if bone:
                out[bone] = np.array(animation['tracks/%d/keys' % i], dtype=np.float64).reshape(-1, 6)
        i += 1
    return out


def depth(parent, bone):
    d = 0
    while bone in parent:
        bone = parent[bone]
        d += 1
    return d


def solve_offsets(parent, godot, gltf):
    """The constant D per bone, and the worst angle in radians by which each bone's own clip then disagrees."""
    offsets, residual = {}, {}
    for bone in sorted(godot, key=lambda b: depth(parent, b)):
        if bone not in gltf:
            continue
        times, values = gltf[bone]
        above = offsets.get(parent.get(bone), np.array([0, 0, 0, 1.0]))
        d = qnorm(qmul(qmul(qinv(sample(godot[bone], times[0])), above), values[0].astype(float)))
        offsets[bone] = d
        worst = 0.0
        for k in range(len(times)):
            predicted = qmul(qmul(qinv(above), sample(godot[bone], times[k])), d)
            worst = max(worst, 2 * np.arccos(min(1.0, abs(np.dot(predicted, values[k])))))
        residual[bone] = worst
    return offsets, residual
