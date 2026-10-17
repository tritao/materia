#!/usr/bin/env python3
"""Finalize a complete force stroke using its short-cut tolerance convergence gate."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fast-directory', required=True)
    parser.add_argument('--fine-stopped-log', help='Optional intentionally stopped original run')
    args = parser.parse_args()
    directory = Path(args.fast_directory).resolve()
    report = json.loads((directory / 'loads.json').read_text())
    calibration = json.loads((directory / 'calibration.json').read_text())
    matches = [r for r in calibration['trials']
               if r['candidate']['toleranceMm'] == report['sweepToleranceMm']
               and r['candidate']['threads'] == report['threads']]
    if len(matches) != 2 or not all(r['passesOnePercent'] for r in matches):
        raise ValueError('Full-run settings must pass both short-cut convergence fixtures')
    if 'Exit status: 0' not in (directory / 'time.log').read_text():
        raise ValueError('Full stroke did not complete successfully')
    report.update(referenceForceAccepted=True, forceToleranceValidated=True,
                  fineFullStrokeValidated=False,
                  acceptanceBasis='completed full stroke; <1% short reference-geometry fixture convergence; no fine full-stroke comparison')
    (directory / 'loads.json').write_text(json.dumps(report, indent=2) + '\n')
    with (directory / 'ratings.log').open('w') as log:
        completed = subprocess.run(['./haxeon/.tools/hashlink/hl', str(directory / 'ratings-main.hl'),
                                   'reference-loads', str(directory / 'loads.json')],
                                  stdout=log, stderr=subprocess.STDOUT, env=os.environ)
    lines = (directory / 'ratings.log').read_text().splitlines()
    def extract(prefix):
        return next((json.loads(line[len(prefix):]) for line in lines if line.startswith(prefix)), None)
    result = dict(referenceForceAccepted=completed.returncode == 0,
                  forceSettings={key: report.get(key) for key in ['threads', 'sweepToleranceMm', 'pathToleranceMm', 'acceptanceBasis']},
                  fineFullStrokeValidated=False,
                  wallSensitivity=extract('MANDREL_SENSITIVITY '),
                  driveRatings=extract('REFERENCE_RATINGS '),
                  fullHB11Budget='open: combined physical source budget remains separate')
    if args.fine_stopped_log:
        result['originalFineRun'] = dict(status='intentionally stopped by user, not a failure',
                                        windows=40400, totalWindows=92151,
                                        partialLog=args.fine_stopped_log)
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
