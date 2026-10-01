#!/usr/bin/env python3
"""Builds the library glTF the runtime loads: the free pack's character and clips, plus the extracted clips.

    build_library.py --base ual-standard.glb --manifest clips.json --clips clips --out ual-work.glb

Every clip file named in the manifest is added to the base file's animations, matched to its bones by name.
Clips the base already carries are left as they are.
"""
import argparse
import hashlib
import json
import os
import struct


def read_glb(path):
    data = open(path, 'rb').read()
    length = struct.unpack_from('<I', data, 12)[0]
    document = json.loads(data[20:20 + length])
    bin_at = 20 + length + 8
    return document, bytearray(data[bin_at:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base', required=True)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--clips', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    base, blob = read_glb(args.base)
    names = {n['name']: i for i, n in enumerate(base['nodes'])}
    have = {a['name'] for a in base['animations']}
    manifest = json.load(open(args.manifest))
    added = 0
    for entry in manifest['clips']:
        if entry['file'] is None or entry['name'] in have:
            continue
        path = os.path.join(args.clips, entry['file'])
        if hashlib.sha256(open(path, 'rb').read()).hexdigest() != entry['sha256']:
            raise SystemExit('%s does not match its manifest entry' % path)
        clip, data = read_glb(path)
        animation = clip['animations'][0]
        views, accessors = len(base['bufferViews']), len(base['accessors'])
        while len(blob) % 4:
            blob.append(0)
        shift = len(blob)
        blob.extend(data)
        for view in clip['bufferViews']:
            base['bufferViews'].append(dict(view, byteOffset=view['byteOffset'] + shift))
        for accessor in clip['accessors']:
            base['accessors'].append(dict(accessor, bufferView=accessor['bufferView'] + views))
        for sampler in animation['samplers']:
            sampler['input'] += accessors
            sampler['output'] += accessors
        for channel in animation['channels']:
            channel['target']['node'] = names[clip['nodes'][channel['target']['node']]['name']]
        base['animations'].append(animation)
        added += 1
    base['buffers'][0].pop('uri', None)
    base['buffers'][0]['byteLength'] = len(blob)
    text = json.dumps(base, separators=(',', ':')).encode()
    text += b' ' * ((4 - len(text) % 4) % 4)
    while len(blob) % 4:
        blob.append(0)
    total = 12 + 8 + len(text) + 8 + len(blob)
    with open(args.out, 'wb') as out:
        out.write(struct.pack('<4sII', b'glTF', 2, total) + struct.pack('<I4s', len(text), b'JSON') + text)
        out.write(struct.pack('<I4s', len(blob), b'BIN\0') + bytes(blob))
    print('added', added, 'clips; library has', len(base['animations']))


if __name__ == '__main__':
    main()
