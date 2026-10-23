"""Audit available control exports without promoting an offline splice to J5.

Read-only to the producer tree. Hash/certificate checks use its pinned reader;
no history is generated, rescaled or relabelled as runtime acceptance.
"""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exports', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.source.resolve()))
    from curved_error import load_export
    rows = []
    for path in sorted(args.exports.glob('*.manifest.json')):
        prefix = path.with_name(path.name.removesuffix('.manifest.json'))
        metadata = prefix.with_suffix('.json')
        if not metadata.is_file():
            continue
        initial = json.loads(metadata.read_text())
        control = initial.get('control')
        if control is None:
            continue
        before = sha(path)
        try:
            meta, samples, manifest = load_export(prefix, path)
            if before != sha(path):
                raise ValueError('Producer manifest changed during audit')
            config = meta['config']
            observed = np.max(np.abs(samples[:, 3:9]), axis=0)
            limits = np.array([2/3, 2/3, 3, 3, config['jerkX'], config['jerkY']])
            if not np.all(observed <= limits*(1+1e-6)+1e-6):
                raise ValueError('Samples exceed unchanged machine V/A/J limits')
            scope = meta.get('controlScope', '')
            offline = 'Offline ordinary compiler prefix/splice/tail' in scope
            # A scope string alone cannot establish the runner's ready-stop,
            # clock-filter, next-window or latency behaviour. Require independent
            # runtime provenance before any future admission route is added.
            rows.append(dict(name=prefix.name, control=control, scope=scope,
                durationS=meta['durationS'], continuousAndSampledBoundsPass=True,
                producerManifestSha256=before, exportSha256=manifest['outputSha256'],
                sourceSha256=manifest.get('sourceSha256'),
                sampledMaxAbs=dict(zip(('vx','vy','ax','ay','jx','jy'), observed.tolist())),
                admittedToJ5=False, classification='offline-splice' if offline else 'runtime-provenance-unqualified'))
        except (ValueError, KeyError, OSError) as error:
            rows.append(dict(name=prefix.name, control=control, admittedToJ5=False,
                classification='invalid-or-changing-export', reason=str(error)))
    if not rows:
        raise ValueError('No controlled exports found; missing coverage is not a pass')
    out = dict(producerRoot=str(args.exports.resolve()),
        readerSha256=sha(args.source/'curved_error.py'), exports=rows,
        approvedEnvelope=dict(hold='ready jerk-limited published stop; planned resume',
            overrideAtOrBelow100Percent='filtered executor clock with derivative V/A/J reserve',
            overrideAbove100Percent='raised-feed next-window replan under unchanged axis V/A/J and feed cap'),
        admittedControlledExports=0,
        missingCoverage=['runtime ready-stop/filtered-clock histories with provenance',
            'above-100-percent raised-feed next-window replans'],
        conclusion='Diagnostic controls only; nominal subset remains the structural acceptance screen. No full-envelope release.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=2)+'\n')
    print(json.dumps(dict(exports=len(rows), validBounds=sum(r.get('continuousAndSampledBoundsPass',False) for r in rows),
        admittedControlledExports=0, classificationCounts={k:sum(r['classification']==k for r in rows)
            for k in sorted({r['classification'] for r in rows})})))


if __name__ == '__main__':
    main()
