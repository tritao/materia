#!/usr/bin/env python3
"""Reuse exact observations only after proving that only the two covers changed.

STEP geometry records must be byte-identical, including schema; only FILE_NAME
metadata may differ. Fixed/module placements, rotor assignments and the tree must
also be identical. Every cover pair is freshly checked at every required pose.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / 'build/straight'
BASELINE = BUILD / 'clearance-baseline'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normalized_step(data):
    if data.count(b'\nDATA;') != 1 or not data.rstrip().endswith(b'END-ISO-10303-21;'):
        raise ValueError('Expected one complete STEP data section')
    header, records = data.split(b'\nDATA;', 1)
    header, count = re.subn(rb"FILE_NAME\s*\(.*?\);", b'FILE_NAME();', header, flags=re.S)
    if count != 1:
        raise ValueError('Expected one STEP FILE_NAME header')
    return header + b'\nDATA;' + records


def load(path):
    return json.loads(path.read_text())


def geometry_proof():
    old, new = load(BASELINE / 'current.json'), load(BUILD / 'current.json')
    old_layout, new_layout = load(Path(old['layoutPath'])), load(Path(new['layoutPath']))
    for field in ('occurrences', 'joints', 'fixedModuleMounts'):
        if old_layout[field] != new_layout[field]:
            raise ValueError('Changed assembly tree/placement: ' + field)
    old_engineering, new_engineering = load(BASELINE / 'engineering-input.json'), load(BUILD / 'engineering-input.json')
    for field in ('axes', 'joints', 'toolFaceMm', 'toolNormal'):
        if old_engineering[field] != new_engineering[field]:
            raise ValueError('Changed exported kinematics: ' + field)
    old_record, new_record = load(Path(old['preparationPath'])), load(Path(new['preparationPath']))
    for current, record in ((old, old_record), (new, new_record)):
        for relative, expected in record['files'].items():
            if digest(Path(current['preparationPath']).parent / relative) != expected:
                raise ValueError('Changed immutable prepared input: ' + relative)
    def metadata(layout):
        return [{k: v for k, v in a.items() if k != 'sha256'} for a in layout['assets']]
    if metadata(old_layout) != metadata(new_layout):
        raise ValueError('Changed source asset selection/partitions')
    changed = {p['asset'] + '.STEP' for p in new_record['parts'] if p['role'] == 'cover'}
    if len(changed) != 2:
        raise ValueError('Expected exactly two cover assets')
    left, right = Path(old['vendorDirectory']), Path(new['vendorDirectory'])
    if {p.name for p in left.iterdir()} != {p.name for p in right.iterdir()}:
        raise ValueError('Changed vendor file set')
    files = {}
    for a in left.iterdir():
        b = right / a.name
        if a.name in changed:
            continue
        before, after = a.read_bytes(), b.read_bytes()
        if a.suffix == '.STEP':
            before, after = normalized_step(before), normalized_step(after)
        if before != after:
            raise ValueError('Changed non-cover geometry/repair: ' + a.name)
        files[a.name] = dict(baselineSha256=digest(a), currentSha256=digest(b),
                            normalizedSha256=hashlib.sha256(before).hexdigest())
    baseline_report = load(BASELINE / 'motion-report.json')
    if baseline_report['layoutSha256'] != digest(Path(old['layoutPath'])) or baseline_report['missingPosesRad']:
        raise ValueError('Incomplete/stale baseline CAD observations')
    for path, expected in baseline_report['inputSha256'].items():
        if digest(BASELINE / Path(path).name) != expected:
            raise ValueError('Changed baseline CAD checkpoint')
    required = load(BUILD / 'required-probes.json')
    if required != load(BASELINE / 'required-probes.json'):
        raise ValueError('Changed probe configurations')
    return dict(status='byte_identical_geometry_except_two_covers',
                baselineLayoutSha256=digest(Path(old['layoutPath'])), layoutSha256=digest(Path(new['layoutPath'])),
                baselineReportSha256=digest(BASELINE / 'motion-report.json'),
                currentPreparationSha256=digest(Path(new['preparationPath'])),
                changedAssets=sorted(changed), unchangedFiles=files,
                producerSha256=digest(Path(__file__)), requiredPoses=len(required))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--collect', action='store_true')
    args = parser.parse_args()
    proof = geometry_proof()
    (BUILD / 'cover-geometry-equivalence.json').write_text(json.dumps(proof, indent=2) + '\n')
    if not args.collect:
        print(proof['status'], len(proof['unchangedFiles']), 'verified unchanged files')
        return
    def key(q):
        return tuple(round(float(x), 12) for x in q)
    required = {key(q) for q in load(BUILD / 'required-probes.json')}
    baseline = {key(r['joints']): r for r in load(BASELINE / 'motion-report.json')['observations']}
    fresh = {}
    paths = [BUILD / ('cover-motion-' + str(i) + '.json') for i in range(2)]
    for path in paths:
        metadata = load(Path(str(path) + '.provenance.json'))
        shard = path.stem[-1]
        records = load(path)
        if (metadata['status'] != 'complete' or metadata['layoutSha256'] != proof['layoutSha256'] or
                metadata['producerSha256'] != digest(ROOT / 'ZeroErrArmPreview.hx') or
                metadata['poseFileSha256'] != digest(BUILD / ('probes-' + shard + '.json')) or
                metadata['observationsSha256'] != digest(path) or metadata['poses'] != len(records)):
            raise ValueError('Incomplete/stale fresh cover checkpoint')
        for row in records:
            if key(row['joints']) in fresh:
                raise ValueError('Duplicate fresh cover observation')
            if any(not (p['a'].endswith('-cover') or p['b'].endswith('-cover'))
                   for p in row['overlaps'] + row['indeterminatePairs']):
                raise ValueError('Fresh cover observation contains unrelated pair')
            fresh[key(row['joints'])] = row
    if set(baseline) != required or set(fresh) != required:
        raise ValueError('Missing baseline or fresh cover probe')
    rows = []
    for q in sorted(required):
        row = dict(joints=fresh[q]['joints'])
        for field in ('overlaps', 'indeterminatePairs'):
            row[field] = [p for p in baseline[q][field] if not (p['a'].endswith('-cover') or p['b'].endswith('-cover'))] + fresh[q][field]
        rows.append(row)
    merged = BUILD / 'cover-regression-observations.json'
    merged.write_text(json.dumps(rows, indent=2) + '\n')
    output = BUILD / 'motion-report.json'
    subprocess.run([sys.executable, str(ROOT / 'review-motion.py'), '--straight', '--required',
                    str(BUILD / 'required-probes.json'), '--output', str(output), str(merged)], check=True)
    report = load(output)
    report.update(provenance='Non-cover pair observations reused after byte-identical STEP geometry/schema and unchanged tree/placements proof; every cover pair freshly checked at all 60 poses.',
                  coverGeometryEquivalenceSha256=digest(BUILD / 'cover-geometry-equivalence.json'),
                  freshCoverCheckpointSha256={str(p): digest(p) for p in paths},
                  baselineReportSha256=proof['baselineReportSha256'])
    output.write_text(json.dumps(report, indent=2) + '\n')
    window = load(ROOT / 'straight-planning-window.json')
    def inside(q):
        return all(bounds[0] <= q[int(joint[1:])-1]*180/3.141592653589793 <= bounds[1]
                   for joint, bounds in window['limitsDeg'].items())
    retained = [r for r in rows if inside(r['joints'])]
    unsafe = [r for r in retained if r['overlaps'] or r['indeterminatePairs']]
    restriction = dict(status='observed_probes_only_not_global_clearance',
                       layoutSha256=proof['layoutSha256'], motionReportSha256=digest(output),
                       planningWindowSha256=digest(ROOT / 'straight-planning-window.json'),
                       retainedPoses=len(retained), excludedPoses=len(rows)-len(retained),
                       unsafeRetainedPoses=unsafe, scope=window['basis'])
    (BUILD / 'planning-window-review.json').write_text(json.dumps(restriction, indent=2) + '\n')
    if unsafe:
        raise ValueError('Planning window retains a known unsafe diagnostic pose')
    print(len(retained), 'retained diagnostic/cycle configurations have no recorded overlap or failed query')


if __name__ == '__main__':
    main()
