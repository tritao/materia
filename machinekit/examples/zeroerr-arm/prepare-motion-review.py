#!/usr/bin/env python3
"""Prepare deterministic diagnostic and quintic-cycle CAD probes for --straight.

Samples are observations, not a continuous-clearance certificate or hardware travel.
"""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    directory = ROOT / 'build/straight'
    current = json.loads((directory / 'current.json').read_text())
    config_path = ROOT / 'revised-engineering-config.json'
    config = json.loads(config_path.read_text())
    ready = [.2, -.8, 1., .1, -.6, .2]
    probes = [[0.] * 6, ready]
    for n in range(1, 13):
        q = []
        for base in (2, 3, 5, 7, 11, 13):
            index, factor, value = n, 1., 0.
            while index:
                factor /= base
                value += factor * (index % base)
                index //= base
            q.append((2 * value - 1) * math.radians(170))
        probes.append(q)
    for joint in range(6):
        for sign in (-1, 1):
            q = ready.copy()
            q[joint] = sign * math.radians(170)
            probes.append(q)
    waypoints = config['missionStudy']['waypointsRad']
    # Seven intervals per segment, including exact endpoints. The quintic is
    # the same stop-to-stop blend used by the engineering load study.
    intervals = config['missionStudy']['samplesPerSegment']
    cycle = []
    for a, b in zip(waypoints, waypoints[1:]):
        for index in range(intervals + 1):
            u = index / intervals
            blend = 10*u**3 - 15*u**4 + 6*u**5
            cycle.append([x + blend*(y-x) for x, y in zip(a, b)])
    unique = []
    seen = set()
    for q in probes + cycle:
        key = tuple(round(value, 12) for value in q)
        if key not in seen:
            seen.add(key)
            unique.append(q)
    for name, rows in [('required-probes', unique), ('cycle-probes', cycle),
                       ('probes-0', unique[::2]), ('probes-1', unique[1::2])]:
        (directory / (name + '.json')).write_text(json.dumps(rows, indent=2) + '\n')
    metadata = {'status': 'prepared_not_checked', 'layoutSha256': digest(Path(current['layoutPath'])),
                'configSha256': digest(config_path), 'generatorSha256': digest(Path(__file__)),
                'uniqueConfigurations': len(unique), 'diagnosticObservations': len(probes),
                'cycleObservations': len(cycle), 'intervalsPerCycleSegment': intervals,
                'probeFiles': {name: digest(directory / name) for name in
                    ('required-probes.json', 'cycle-probes.json', 'probes-0.json', 'probes-1.json')},
                'scope': 'Uncommissioned diagnostic range plus synthetic tending intermediates; no real cell or swept proof'}
    (directory / 'probe-preparation.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
