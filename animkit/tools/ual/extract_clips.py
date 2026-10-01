#!/usr/bin/env python3
"""Extracts individual clips of the Universal Animation Library as one glTF file each, with provenance.

    extract_clips.py --pck index.pck --free ual-free.gltf --out clips --manifest clips.json NAME [NAME ...]

The library's animations are read from the Godot pack its viewer ships (UAL1_Source and UAL2_Source), retargeted
onto the glTF rig of the free pack (see ual_retarget.py), and written one per file: the rig's bones and a single
animation, no mesh. Each file and the manifest record where the clip came from and what was done to it.
A clip the free glTF already carries is not extracted: that file is the original and is used as it is.
"""
import argparse
import hashlib
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import godot_resource as godot
import ual_retarget as rt

LIBRARIES = {'UAL1': 'UAL1_Source.res', 'UAL2': 'UAL2_Source.gltf'}


def load_libraries(pck):
    files = godot.read_pck(pck)
    out = {}
    for library, marker in LIBRARIES.items():
        blob = [data for name, data in files.items() if marker in name and ('exported' in name or 'imported' in name)][0]
        out[library] = godot.animations(blob)
    return out


def hip_height(animation):
    """The mean height of the hips over a clip."""
    i = 0
    while 'tracks/%d/type' % i in animation:
        if animation['tracks/%d/type' % i] == 'position_3d' and animation['tracks/%d/path' % i].endswith(':Hips'):
            return float(np.array(animation['tracks/%d/keys' % i]).reshape(-1, 5)[:, 3].mean())
        i += 1
    raise SystemExit('no hips position track')


def convert(animation, gltf, offsets, hip_scale):
    """Channels for one Godot animation on the glTF rig: (node, path, times, values) tuples."""
    channels = []
    i = 0
    while 'tracks/%d/type' % i in animation:
        kind = animation['tracks/%d/type' % i]
        bone = rt.godot_to_def(animation['tracks/%d/path' % i])
        i += 1
        if bone not in gltf.index or bone not in offsets or kind not in ('rotation_3d', 'position_3d'):
            continue
        above = offsets.get(gltf.parent.get(bone), np.array([0, 0, 0, 1.0]))
        if kind == 'rotation_3d':
            keys = np.array(animation['tracks/%d/keys' % (i - 1)], dtype=np.float64).reshape(-1, 6)
            values = np.array([rt.qnorm(rt.qmul(rt.qmul(rt.qinv(above), q), offsets[bone])) for q in keys[:, 2:6]])
            for n in range(1, len(values)):
                if np.dot(values[n], values[n - 1]) < 0:
                    values[n] = -values[n]
            channels.append((bone, 'rotation', keys[:, 0], values))
        else:
            keys = np.array(animation['tracks/%d/keys' % (i - 1)], dtype=np.float64).reshape(-1, 5)
            scale = hip_scale if bone == 'DEF-hips' else 1.0
            values = np.array([rt.rotate(rt.qinv(above), v) * scale for v in keys[:, 2:5]])
            channels.append((bone, 'translation', keys[:, 0], values))
    return channels


def write_clip(path, gltf, name, channels, extras):
    blob = bytearray()
    views, accessors, samplers, targets = [], [], [], []

    def add(array, kind):
        array = np.ascontiguousarray(array, dtype='<f4')
        while len(blob) % 4:
            blob.append(0)
        views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': array.nbytes})
        blob.extend(array.tobytes())
        accessor = {'bufferView': len(views) - 1, 'componentType': 5126, 'count': array.shape[0], 'type': kind}
        if kind == 'SCALAR':
            accessor['min'], accessor['max'] = [float(array.min())], [float(array.max())]
        accessors.append(accessor)
        return len(accessors) - 1

    for bone, kind, times, values in channels:
        samplers.append({'input': add(times.reshape(-1, 1), 'SCALAR'),
                         'output': add(values, 'VEC4' if kind == 'rotation' else 'VEC3'), 'interpolation': 'LINEAR'})
        targets.append({'sampler': len(samplers) - 1, 'target': {'node': gltf.index[bone], 'path': kind}})
    nodes = [{k: v for k, v in n.items() if k not in ('mesh', 'skin')} for n in gltf.nodes]
    document = {'asset': {'version': '2.0', 'generator': 'animkit/tools/ual/extract_clips.py',
                          'copyright': 'Quaternius, Universal Animation Library (CC0)', 'extras': extras},
                'scene': 0, 'scenes': [{'nodes': gltf.json['scenes'][0]['nodes']}], 'nodes': nodes,
                'animations': [{'name': name, 'channels': targets, 'samplers': samplers, 'extras': extras}],
                'accessors': accessors, 'bufferViews': views, 'buffers': [{'byteLength': len(blob)}]}
    text = json.dumps(document, separators=(',', ':')).encode()
    text += b' ' * ((4 - len(text) % 4) % 4)
    while len(blob) % 4:
        blob.append(0)
    total = 12 + 8 + len(text) + 8 + len(blob)
    with open(path, 'wb') as out:
        out.write(struct.pack('<4sII', b'glTF', 2, total) + struct.pack('<I4s', len(text), b'JSON') + text)
        out.write(struct.pack('<I4s', len(blob), b'BIN\0') + bytes(blob))
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pck', required=True)
    parser.add_argument('--free', required=True, help='the free pack .gltf (its .bin beside it)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('clips', nargs='+')
    args = parser.parse_args()
    gltf = rt.Gltf(args.free, os.path.splitext(args.free)[0] + '.bin')
    free = {a['name'] for a in gltf.json['animations']}
    libraries = load_libraries(args.pck)
    reference = libraries['UAL1']['PickUp_Table']
    offsets, residual = rt.solve_offsets(gltf.parent, rt.godot_rotations(reference), gltf.rotations('PickUp_Table'))
    median = float(np.median(list(residual.values())))
    # The glTF rig is the Godot skeleton scaled down (its hips rest lower over the same feet), so a Godot hips
    # translation is scaled by the same factor. Idle_Loop is in both forms; the ratio of their mean hip travel gives it.
    idle = gltf.json['animations']
    idle_channel = [c for a in idle if a['name'] == 'Idle_Loop' for c in a['channels']
                    if gltf.nodes[c['target']['node']]['name'] == 'DEF-hips' and c['target']['path'] == 'translation'][0]
    idle_sampler = [a for a in idle if a['name'] == 'Idle_Loop'][0]['samplers'][idle_channel['sampler']]
    gltf_hips = np.linalg.norm(gltf.accessor(idle_sampler['output']).mean(axis=0))
    godot_hips = np.linalg.norm(np.array(next(
        libraries['UAL1']['Idle_Loop']['tracks/%d/keys' % n] for n in range(200)
        if libraries['UAL1']['Idle_Loop'].get('tracks/%d/path' % n, '').endswith(':Hips')
        and libraries['UAL1']['Idle_Loop']['tracks/%d/type' % n] == 'position_3d')).reshape(-1, 5)[:, 2:5].mean(axis=0))
    hip_scale = float(gltf_hips / godot_hips)
    manifest = {'retarget': {'reference_clip': 'PickUp_Table', 'bones': len(offsets), 'hip_scale': round(hip_scale, 4),
                             'median_error_degrees': round(float(np.degrees(median)), 3),
                             'worst_error_degrees': round(float(np.degrees(max(residual.values()))), 3)}, 'clips': []}
    for name in args.clips:
        if name in free:
            manifest['clips'].append({'name': name, 'file': None, 'source': 'ual-standard.glb (free glTF, original)'})
            continue
        library = 'UAL1' if name in libraries['UAL1'] else 'UAL2'
        animation = libraries[library][name]
        channels = convert(animation, gltf, offsets, hip_scale)
        extras = {'name': name, 'library': 'Universal Animation Library ' + ('1' if library == 'UAL1' else '2'),
                  'source': 'Godot animation "%s" in %s_Source of the library viewer pack' % (name, library),
                  'license': 'CC0', 'retargeted_to': 'ual-standard.glb rig (DEF- bones)',
                  'hip_height_scale': round(hip_scale, 4), 'length_seconds': round(float(animation['length']), 4)}
        os.makedirs(os.path.join(args.out, library), exist_ok=True)
        relative = '%s/%s.glb' % (library, name)
        digest = write_clip(os.path.join(args.out, relative), gltf, name, channels, extras)
        manifest['clips'].append(dict(extras, file=relative, sha256=digest, channels=len(channels)))
    json.dump(manifest, open(args.manifest, 'w'), indent=1)
    print('wrote', len([c for c in manifest['clips'] if c['file']]), 'clips;', manifest['retarget'])


if __name__ == '__main__':
    main()
