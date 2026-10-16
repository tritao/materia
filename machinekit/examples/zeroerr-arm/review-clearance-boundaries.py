#!/usr/bin/env python3
"""Compare conditional enclosing-hull slices and propose a simulation planning window.

The window excludes observed folds; collision checking remains required inside it.
"""
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build/straight"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interval(scan):
    rows = {sample['angleDeg']: sample['observation'] for sample in scan['samples']}
    for boundary in scan['boundaries']:
        rows.update({boundary['lowDeg']: boundary['low'], boundary['highDeg']: boundary['high']})
    ordered = sorted(rows)
    center = ordered.index(0)
    if not rows[0]['clear']:
        raise ValueError('Slice has no clear zero-angle centre: ' + scan['id'])
    left = right = center
    while left > 0 and rows[ordered[left - 1]]['clear']:
        left -= 1
    while right < len(ordered) - 1 and rows[ordered[right + 1]]['clear']:
        right += 1
    return [ordered[left], ordered[right]]


def main():
    config_path = ROOT / 'clearance-review-config.json'
    config = json.loads(config_path.read_text())
    current = json.loads((BUILD / 'current.json').read_text())
    reports = [json.loads((BUILD / ('clearance-boundaries-' + name + '.json')).read_text())
               for name in ('baseline', 'revised')]
    for report in reports:
        if report['status'] != 'conditional_slices_checked' or report['configSha256'] != digest(config_path):
            raise ValueError('Incomplete/stale conditional review')
        if report['producerSha256'] != digest(ROOT.parent / 'reference-arms/StraightBoundaryReview.hx'):
            raise ValueError('Changed hull boundary producer')
        if report['method'] != 'enclosing_visual_hulls' or report['marginMm'] != config['marginMm']:
            raise ValueError('Changed hull method or clearance margin')
        if [s['id'] for s in report['scans']] != [s['id'] for s in config['scans']]:
            raise ValueError('Missing conditional slice')
        for scan, definition in zip(report['scans'], config['scans']):
            if scan['pairs'] != definition['pairs'] or scan['seedRad'] != definition['seedRad']:
                raise ValueError('Changed slice definition')
            observations = [s['observation'] for s in scan['samples']]
            observations += [b[side] for b in scan['boundaries'] for side in ('low', 'high')]
            if any('error' in pair for observation in observations for pair in observation['pairs']):
                raise ValueError('Indeterminate hull slice: ' + scan['id'])
    if reports[0]['layoutSha256'] != config['baselineLayoutSha256']:
        raise ValueError('Wrong baseline geometry')
    if reports[1]['layoutSha256'] != digest(Path(current['layoutPath'])):
        raise ValueError('Stale revised geometry')
    rows = []
    for before, after in zip(reports[0]['scans'], reports[1]['scans']):
        rows.append(dict(id=after['id'], joint=after['joint'],
                         baselineObservedIntervalDeg=interval(before),
                         revisedObservedIntervalDeg=interval(after)))
    proposal = {}
    for joint in (3, 4):
        intervals = [r['revisedObservedIntervalDeg'] for r in rows if r['joint'] == joint]
        lower = math.ceil(max(i[0] for i in intervals) + 2)
        upper = math.floor(min(i[1] for i in intervals) - 2)
        if not lower < 0 < upper:
            raise ValueError('No usable conditional slice intersection')
        proposal['j' + str(joint)] = [lower, upper]
    result = dict(status='conditional_geometry_review_not_global_clearance',
                  layoutSha256=reports[1]['layoutSha256'], configSha256=digest(config_path),
                  reportsSha256={name: digest(BUILD / ('clearance-boundaries-' + name + '.json'))
                                 for name in ('baseline', 'revised')},
                  analysisSha256=digest(Path(__file__)), slices=rows,
                  proposedPlanningWindowDeg=proposal, angularReserveDeg=2,
                  marginMm=config['marginMm'], hardwareQualified=False,
                  requiresPoseAndContinuousCollisionChecks=True,
                  scope='Coarse conditional enclosing-hull slices, refined observed clearance transitions. '
                        'Unobserved collisions between samples and at other joint combinations remain possible. '
                        'Planning window is an additional restriction, not a globally collision-free envelope.')
    (BUILD / 'clearance-improvement.json').write_text(json.dumps(result, indent=2) + '\n')
    window = dict(schema=1, status='simulation_planning_restriction_only',
                  layoutSha256=result['layoutSha256'], reviewConfigSha256=digest(config_path),
                  limitsDeg=proposal, angularReserveDeg=2, hardwareQualified=False,
                  requiresPoseAndContinuousCollisionChecks=True,
                  basis='Intersection of the sampled conditional J3/J4 intervals around zero, '
                        'rounded inward with 2 degrees reserve. Does not prove other joint combinations clear.')
    (BUILD / 'planning-window-proposal.json').write_text(json.dumps(window, indent=2) + '\n')
    fig, axes = plt.subplots(2, 1, figsize=(12, 10), sharex=True, layout='constrained')
    for ax, report, title in zip(axes, reports, ('Original geometry', 'Revised geometry')):
        for i, scan in enumerate(report['scans']):
            samples = scan['samples']
            for sample in samples:
                ax.scatter(sample['angleDeg'], i, s=24, color='#29845b' if sample['observation']['clear'] else '#bf493e')
            lo, hi = interval(scan)
            ax.plot([lo, hi], [i, i], color='#29845b', linewidth=3, alpha=.45)
            for boundary in scan['boundaries']:
                angle = (boundary['lowDeg'] + boundary['highDeg']) / 2
                ax.plot([angle, angle], [i - .22, i + .22], color='#333333', linewidth=1.2)
        ax.set_yticks(range(len(report['scans'])), [s['id'] for s in report['scans']], fontsize=8)
        ax.set_title(title)
        ax.grid(axis='x', alpha=.2)
        ax.set_xlim(-185, 185)
    axes[-1].set_xlabel('Swept joint angle (degrees); J3 for elbow slices, J4 for wrist slices')
    fig.suptitle('Known interference pairs and distal fasteners · enclosing hulls · 0.5 mm margin')
    axes[0].legend(handles=[Patch(color='#29845b', label='Hull distance meets margin'),
                            Patch(color='#bf493e', label='Hull overlap / below margin')], loc='upper right', fontsize=8)
    fig.savefig(BUILD / 'clearance-boundaries.png', dpi=160)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
