#!/usr/bin/env python3
"""Collect structural observations without treating collisions or failed pairs as clear."""
import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

def key(q):
    if len(q) != 6:
        raise ValueError('Motion observations require six joint coordinates')
    # The same mathematical Halton angles can differ in their last printed bit.
    return tuple(round(float(x), 12) for x in q)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--required', type=Path)
    parser.add_argument('--straight', action='store_true')
    parser.add_argument('--original', action='store_true')
    parser.add_argument('--output', type=Path, default=HERE/'build/revised/motion-report.json')
    args = parser.parse_args()
    if args.original and args.straight:
        parser.error('Choose original or straight geometry')
    observations = {}
    for path in args.inputs:
        for row in json.loads(path.read_text()):
            if not all(field in row for field in ('joints', 'overlaps', 'indeterminatePairs')):
                raise ValueError('Incomplete observation in '+str(path))
            target = observations.setdefault(key(row['joints']), dict(joints=row['joints'],overlaps=[],indeterminatePairs=[]))
            # Preserve every positive/indeterminate observation across repeated probes.
            for field in ('overlaps', 'indeterminatePairs'):
                for item in row.get(field, []):
                    if item not in target[field]:
                        target[field].append(item)
    required = {key(q) for q in json.loads(args.required.read_text())} if args.required else set(observations)
    missing = sorted(required-set(observations))
    rows = list(observations.values())
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    layout = HERE/'revised-layout.json'
    engineering = HERE/'build/revised/engineering-input.json'
    if args.original:
        layout = HERE/'reference-layout.json'
        engineering = HERE/'build/engineering-input.json'
    if args.straight:
        current = json.loads((HERE/'build/straight/current.json').read_text())
        layout = Path(current['layoutPath'])
        engineering = HERE/'build/straight/engineering-input.json'
        prepared = json.loads((HERE/'build/straight/probe-preparation.json').read_text())
        if prepared['layoutSha256'] != digest(layout):
            raise ValueError('Probe preparation refers to an older straight layout')
        for name, expected in prepared['probeFiles'].items():
            if digest(HERE/'build/straight'/name) != expected:
                raise ValueError('Changed prepared probe file: '+name)
    result = dict(status='incomplete' if missing else 'complete_with_exclusions' if any(
            r['overlaps'] or r['indeterminatePairs'] for r in rows) else 'sampled_structures_clear',
        scope=('22 original structural solids and three output spigots already included' if args.original else
               '29 structural solids including adapters/covers and six output spigots' if args.straight else
               '25 structural solids including six output spigots')+' at discrete exploratory poses. '
              'Not swept clearance, all detail fasteners, cable clearance, hardware travel, or real cell reach.',
        uniqueRequiredPoses=len(required),uniqueObservedPoses=len(rows),missingPosesRad=missing,
        clearPoses=sum(not r['overlaps'] and not r['indeterminatePairs'] for r in rows),
        collisionPoses=sum(bool(r['overlaps']) for r in rows),
        indeterminatePoses=sum(bool(r['indeterminatePairs']) for r in rows),
        inputSha256={str(p):digest(p) for p in args.inputs},
        layoutSha256=digest(layout),
        engineeringInputSha256=digest(engineering),
        provenance='Current original-reference checkpoint files.' if args.original else 'Current straight-variant checkpoint files.' if args.straight else 'Collected within this review. Completed legacy clear rows can originate from the '
                   'fail-closed initial logs; positive and failed witnesses are rechecked for pair details.',
        observations=rows)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(f"{result['status']}: {len(rows)}/{len(required)} unique poses; "
          f"{result['clearPoses']} clear, {result['collisionPoses']} colliding, "
          f"{result['indeterminatePoses']} with indeterminate pairs")
    if missing:
        raise SystemExit('Required probes remain unobserved')

if __name__=='__main__':main()
