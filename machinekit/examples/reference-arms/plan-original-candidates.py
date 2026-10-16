#!/usr/bin/env python3
"""Run independent original-arm base searches with the same native planner settings.

Build haxeon.json first. Workers write separate reports and never publish previews.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / 'build/cell-registration'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def count(candidate):
    return sum(bool((r.get('continuous') or {}).get('clear')) for r in candidate['routes'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--adopt-first-progress', action='store_true')
    parser.add_argument('--merge-only', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 3:
        parser.error('Use one to three independent workers')
    module = ROOT / 'build/host/main.hl'
    bytecode = digest(module)
    scan_path = BUILD / 'original-registration-review.json'
    scan = read(scan_path)
    if len(scan['models']) != 1 or len(scan['models'][0]['candidates']) != 9:
        raise ValueError('Need all nine original base candidates first')
    def run(index):
        log = BUILD / f'original-candidate{index}.log'
        with log.open('w') as output:
            subprocess.run(['bash', str(ROOT / 'run-built.sh'), '--registration', '--original-reference', '--plan-transfers',
                            '--plan-candidate-index='+str(index)], stdout=output, stderr=subprocess.STDOUT, check=True)
        print('Completed original base rank', index, flush=True)
    indices = list(range(1 if args.adopt_first_progress else 0, 9))
    if not args.merge_only:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            list(pool.map(run, indices))
    if digest(module) != bytecode:
        raise ValueError('Native review bytecode changed during independent searches')
    candidates, evidence, template = [], {}, None
    if args.adopt_first_progress:
        path = BUILD / 'zeroerr-original-planned-progress.json'
        progress = read(path)
        if any(progress[k] != scan[k] for k in ('fixtureSha256', 'configSha256')) or progress['sourceFingerprint'] != scan['models'][0]['sourceFingerprint']:
            raise ValueError('Changed sequential first-candidate evidence')
        candidates.append(progress['candidates'][0])
        evidence[str(path)] = digest(path)
    for index in indices:
        path = BUILD / f'original-registration-candidate{index}-planned.json'
        report = read(path)
        if any(report[k] != scan[k] for k in ('fixtureSha256', 'configSha256')) or report['models'][0]['sourceFingerprint'] != scan['models'][0]['sourceFingerprint']:
            raise ValueError('Mismatched independent base search')
        if len(report['models']) != 1 or len(report['models'][0]['candidates']) != 1:
            raise ValueError('Incomplete independent base report')
        template = report
        candidates += report['models'][0]['candidates']
        evidence[str(path)] = digest(path)
    def position(candidate):
        return tuple(round(candidate['baseWorldMetres'][k], 8) for k in 'xyz')
    if len(candidates) != 9 or {position(c) for c in candidates} != {position(c) for c in scan['models'][0]['candidates']}:
        raise ValueError('Duplicate/missing base search')
    for candidate in candidates:
        if len(candidate['routes']) != 23 or len(candidate['poses']) != 82:
            raise ValueError('Incomplete transfer/target coverage')
    candidates.sort(key=lambda c: (count(c), c['clearPoses']), reverse=True)
    model = template['models'][0]
    model['candidates'] = [candidates[0]]
    model['examinedCandidates'] = [dict(baseWorldMetres=c['baseWorldMetres'],continuousRoutes=count(c),
                                       clearPoses=c['clearPoses'],reviewedRoutes=c['reviewedRoutes']) for c in candidates]
    template['independentSearchEvidenceSha256'] = evidence
    template['independentWorkerBytecodeSha256'] = bytecode
    template['bytecodeScope'] = 'Independent workers; an adopted sequential checkpoint is bound separately by its evidence hash.'
    template['candidateScanSha256'] = digest(scan_path)
    template['mergeProducerSha256'] = digest(Path(__file__))
    (BUILD / 'original-registration-planned.json').write_text(json.dumps(template, indent=2)+'\n')
    print('Selected original base:', candidates[0]['baseWorldMetres'], count(candidates[0]), '/ 23 continuously checked transfers')


if __name__ == '__main__':
    main()
