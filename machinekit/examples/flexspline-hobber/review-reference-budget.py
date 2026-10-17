#!/usr/bin/env python3
"""Review saved HB11 inspections without starting or changing a physical cut."""
import argparse
import json
import math
from pathlib import Path
import time

PARAMETERS = ['fp', 'Fp', 'Fr', 'Falpha', 'ffalpha', 'fHalpha', 'Fbeta', 'ffbeta', 'fHbeta']


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None  # A running cut may still be writing its completed inspection.


def atomic_write(path, text):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(text)
    temporary.replace(path)


def review(directory):
    settings = read_json(directory / 'settings.json')
    if settings is None:
        return dict(status='pending', reason='settings not available', numericalBudgetFits=None, hb12AcceptanceComplete=False)
    source_ids = settings['sourceIds']
    labels = ['baseline', *source_ids, 'combined']
    cases = {}
    baseline_thickness = None
    for label in labels:
        raw = read_json(directory / f'{label}.json')
        if raw is None:
            continue
        values = raw['values']
        if any(not isinstance(values.get(k), (float, int)) or not math.isfinite(values[k]) for k in PARAMETERS):
            raise ValueError(f'{label}: incomplete or non-finite metrology parameters')
        thickness = raw.get('thicknessMm', [])
        if label == 'baseline':
            baseline_thickness = thickness
        mean_shift = None
        if baseline_thickness and len(thickness) == len(baseline_thickness):
            mean_shift = sum((a-b)*1000 for a, b in zip(thickness, baseline_thickness))/len(thickness)
        cases[label] = dict(valuesUm=values, grades=raw['grades'], warnings=raw['warnings'],
                            extrapolation=raw['extrapolation'], profileSourcePoints=raw['profileSourcePoints'],
                            helixSourcePoints=raw['helixSourcePoints'],
                            meanThicknessShiftFromBaselineUm=mean_shift)
    final = read_json(directory / 'result.json')
    status_path = directory / 'all-exit-status.txt'
    exit_code = int(status_path.read_text().strip()) if status_path.exists() else None
    complete = len(cases) == len(labels) and final is not None
    dominant = {}
    if 'baseline' in cases:
        for parameter in PARAMETERS:
            rows = [(abs(cases[label]['valuesUm'][parameter]-cases['baseline']['valuesUm'][parameter]), label)
                    for label in source_ids if label in cases]
            if rows:
                magnitude, label = max(rows)
                dominant[parameter] = dict(source=label if magnitude > 1e-9 else 'none', absoluteDeltaUm=magnitude)
    failures = None
    if 'combined' in cases:
        failures = [k for k in PARAMETERS if cases['combined']['grades'].get(k) is None
                    or cases['combined']['grades'][k] > 6]
    if final is not None:
        failures = sorted(set((failures or []) + final['failedClass6Parameters']))
    if complete:
        for key, row in dominant.items():
            row['source'] = final['dominant'][key]
    warnings = list(dict.fromkeys(w for case in cases.values() for w in case['warnings']))
    fits = complete and exit_code == 0 and not failures and final.get('additive') is True
    return dict(status='completed' if complete and exit_code == 0 else 'stopped or failed' if exit_code is not None else 'running',
                completedCases=len(cases), expectedCases=len(labels), missingCases=[k for k in labels if k not in cases],
                executionExitCode=exit_code, numericalBudgetFits=bool(fits) if complete else None,
                failedClass6Parameters=failures, traceSuperpositionPassed=None if final is None else final['additive'],
                interactionUm=None if final is None else final['interactionUm'],
                dominantSources=dominant, rankingBasis='absolute signed-parameter delta from baseline; provisional until all source cases finish',
                cases=cases, coverageWarnings=warnings,
                engineeringAcceptance='pending completed source-signature and coverage review; assumed support/contact, unloaded servo and frozen-force inputs qualify the result and do not automatically fail it',
                hb12AcceptanceComplete=False,
                hb12Remaining=['review completed HB11 source signatures and combined result',
                               'reference specification and cheaper-spec comparison',
                               'm0.5 comparison with tightened parts',
                               'measured total cycle breakdown <=180 s'],
                scope=settings['scope'],
                settings={k: settings[k] for k in ['threads', 'sweepToleranceMm', 'pathToleranceMm', 'radialStepMm', 'axialStepMm']})


def markdown(result):
    lines = [f"HB11 reference review: {result['status']}", '',
             f"Completed cases: {result.get('completedCases', 0)}/{result.get('expectedCases', 15)}.",
             f"Numerical budget fits: {result.get('numericalBudgetFits')}; HB12 acceptance complete: false.", '']
    if result.get('failedClass6Parameters'):
        lines += ['Parameters exceeding extrapolated class 6: ' + ', '.join(result['failedClass6Parameters']), '']
    if result.get('dominantSources'):
        lines += ['| Parameter | Dominant source so far | Absolute delta (µm) |', '|---|---|---:|']
        for key, row in result['dominantSources'].items():
            lines.append(f"| {key} | {row['source']} | {row['absoluteDeltaUm']:.6g} |")
        lines += ['', result['rankingBasis'], '']
    for label, case in result.get('cases', {}).items():
        lines += [f"{label}: " + ', '.join(f"{k}={case['valuesUm'][k]:.6g} µm (grade {case['grades'].get(k)})" for k in PARAMETERS), '']
    if result.get('coverageWarnings'):
        lines += ['Coverage/inspection warnings:', ''] + [f'- {w}' for w in result['coverageWarnings']] + ['']
    lines += [result.get('engineeringAcceptance', result.get('reason', 'pending')), '',
              'Module 0.3 grades are engineering extrapolations, not ISO certification.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--watch', action='store_true', help='Update on completed artifacts; exit when the batch exits')
    args = parser.parse_args()
    previous = None
    while True:
        signature = [(p.name, p.stat().st_mtime_ns, p.stat().st_size)
                     for p in sorted(args.directory.glob('*.json')) if p.name not in ['inputs.json', 'review.json']]
        status = args.directory / 'all-exit-status.txt'
        if status.exists():
            signature.append(('exit', status.stat().st_mtime_ns, status.stat().st_size))
        if signature != previous:
            result = review(args.directory)
            atomic_write(args.directory / 'review.json', json.dumps(result, indent=2)+'\n')
            atomic_write(args.directory / 'review.md', markdown(result))
            print(f"HB11_REVIEW {result['status']}: {result.get('completedCases', 0)} cases; numericalBudgetFits={result.get('numericalBudgetFits')}", flush=True)
            previous = signature
        if not args.watch or status.exists():
            return 0
        time.sleep(15)


if __name__ == '__main__':
    raise SystemExit(main())
