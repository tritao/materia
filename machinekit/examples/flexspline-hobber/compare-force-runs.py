#!/usr/bin/env python3
"""Validate an optimized force report against the retained full-stroke authority."""
import argparse
import json
from pathlib import Path


def read_report(path):
    text = Path(path).read_text()
    if text.lstrip().startswith('{'):
        return json.loads(text)
    records = [json.loads(line.split(' ', 1)[1]) for line in text.splitlines()
               if line.startswith('REFERENCE_FORCE ')]
    if not records:
        raise ValueError(f'No completed reference force report in {path}')
    report = records[-1]
    if 'wallSeconds' not in report:
        elapsed = next((line.rsplit(': ', 1)[1] for line in text.splitlines() if 'Elapsed (wall clock)' in line), None)
        if elapsed:
            report['wallSeconds'] = sum(float(part) * 60**power for power, part in enumerate(reversed(elapsed.split(':'))))
    return report


def compare(fine, trial, tolerance=0.01):
    if fine['scope'] != trial['scope'] or fine['windows'] != trial['windows']:
        raise ValueError('Full-stroke scope/window count differs')
    if abs(fine['seconds'] - trial['seconds']) > 1e-9:
        raise ValueError('Reference feed duration differs')
    differences = {}
    for key in ['forceN', 'radialN', 'meanRadialN', 'volumeMm3']:
        reference, value = fine[key], trial[key]
        if reference <= 0:
            raise ValueError(f'Invalid fine {key}')
        differences[key] = abs(value / reference - 1)
    passed = all(value < tolerance for value in differences.values())
    result = {'passed': passed, 'relativeCriterion': tolerance,
              'relativeChanges': differences,
              'components': {'forceN': 'peak tangential window mean',
                             'radialN': 'peak radial window mean',
                             'meanRadialN': 'time-mean radial load, including air'},
              'fineSettings': {key: fine.get(key) for key in ['threads', 'sweepToleranceMm', 'pathToleranceMm']},
              'trialSettings': {key: trial.get(key) for key in ['threads', 'sweepToleranceMm', 'pathToleranceMm']},
              'variationQualification': 'Fine legacy run did not retain per-pass history. Peak/mean agreement does not prove pointwise or per-pass variation agreement; short-cut variation convergence is separate.'}
    if fine.get('wallSeconds') and trial.get('wallSeconds'):
        result['wallSpeedup'] = fine['wallSeconds'] / trial['wallSeconds']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fine')
    parser.add_argument('trial')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    fine, trial = read_report(args.fine), read_report(args.trial)
    # This known archived reference ran at one thread / 0.05 um. Do not infer
    # settings for arbitrary reports: the caller supplies its provenance sidecar.
    result = compare(fine, trial)
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
