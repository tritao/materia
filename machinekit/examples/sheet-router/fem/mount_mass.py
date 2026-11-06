"""Audit built mount mass/inertia against the frozen packet (no structural solve).

Both ledgers retain their component ownership. Stock beams used by the global
model must replace, rather than supplement, their aggregate weldment mass.
All tensors are centroidal, world-oriented, in kg mm².
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tensor(row):
    t = row['centroidalInertiaKgMm2World']
    if t is None:
        raise ValueError('Missing carried-body inertia: ' + row['id'])
    a, b, c, d, e, f = (t[k] for k in ('xx', 'yy', 'zz', 'xy', 'xz', 'yz'))
    if not all(math.isfinite(v) for v in (a, b, c, d, e, f)):
        raise ValueError('Nonfinite inertia: ' + row['id'])
    # Sylvester's criterion checks the complete tensor, including products.
    if min(a, a*b-d*d, a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d) <= 0:
        raise ValueError('Non-positive inertia: ' + row['id'])
    return [[a, d, e], [d, b, f], [e, f, c]]


def aggregate(rows, shift=None):
    rows = list(rows)
    if not rows:
        raise ValueError('Empty mass group')
    mass = sum(r['massKg'] for r in rows)
    if any(not math.isfinite(r['massKg']) or r['massKg'] <= 0 for r in rows):
        raise ValueError('Invalid mass')
    points = [[r['centreOfMassMm'][a] + (shift or {}).get(a, 0.) for a in 'xyz'] for r in rows]
    centre = [sum(r['massKg']*p[i] for r, p in zip(rows, points))/mass for i in range(3)]
    inertia = [[0.]*3 for _ in range(3)]
    for row, point in zip(rows, points):
        local = tensor(row)
        delta = [point[i]-centre[i] for i in range(3)]
        radius2 = sum(v*v for v in delta)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += local[i][j] + row['massKg']*((radius2 if i == j else 0.)-delta[i]*delta[j])
    return dict(massKg=mass, centreOfMassMm=dict(zip('xyz', centre)),
                centroidalInertiaKgMm2World=inertia, bodyCount=len(rows))


def compare(built_path, frozen_path):
    built, frozen = (json.loads(p.read_text()) for p in (built_path, frozen_path))
    parts = {p['id']: p for p in built['parts']}
    if len(parts) != len(built['parts']):
        raise ValueError('Duplicate body mass ownership')
    for row in parts.values():
        tensor(row)
    if not math.isclose(sum(r['massKg'] for r in parts.values()), built['structuralBodyMassKg'], abs_tol=1e-7):
        raise ValueError('Carried mass ledger does not balance')
    if not math.isclose(built['structuralBodyMassKg']+built['independentFloorMassKg'], built['totalMassKg'], abs_tol=1e-7):
        raise ValueError('Floor/carried mass ledger does not balance')
    floor_ids = {p['id'] for p in built['independentFloorParts']}
    old = {p['id']: p for p in frozen['parts']}
    x_ids = {r['id'].split(' (whole chain upper bound)')[0]
             for a in frozen['axes'] if a['axis'] == 'x' for r in a['movingMembers']}
    # Only X-carried bodies move when changing the reference from X800 to X0;
    # this includes them when aggregating the complete Y-moving assembly.
    for name in x_ids:
        old[name]['centreOfMassMm']['x'] -= 800.
    axes = []
    for moving in built['moving']:
        axis = moving['axis']
        ids = [r['id'].split(' (whole chain upper bound)')[0] for r in moving['mass']['members']]
        if len(ids) != len(set(ids)) or any(i not in parts for i in ids):
            raise ValueError('Missing or duplicate moving body: '+axis)
        current = aggregate(parts[i] for i in ids)
        if not math.isclose(current['massKg'], moving['mass']['kg'], abs_tol=1e-7):
            raise ValueError('Moving mass ownership mismatch: '+axis)
        previous = next(a for a in frozen['axes'] if a['axis'] == axis)
        old_ids = [r['id'].split(' (whole chain upper bound)')[0] for r in previous['movingMembers']]
        # Frozen packet was exported at X800; built export is at X0. Compare
        # both centres at the built pose without changing centroidal tensors.
        old_group = aggregate(old[i] for i in old_ids)
        changes = []
        for name in sorted(set(ids) | set(old_ids)):
            before = old[name]['massKg'] if name in old_ids else 0.
            after = parts[name]['massKg'] if name in ids else 0.
            if abs(after-before) > 1e-9:
                changes.append(dict(id=name, beforeKg=before, afterKg=after, deltaKg=after-before))
        axes.append(dict(axis=axis, built=current, frozen=old_group,
                         deltaMassKg=current['massKg']-old_group['massKg'], changes=changes))
    return dict(status='CAD mass/inertia audit; no dynamic acceptance', bedMm=built['bedMm'],
                inputs=[dict(path=str(p), sha256=digest(p)) for p in (built_path, frozen_path)],
                poseMm=built['poseMm'], carriedBodiesChecked=len(parts), independentFloorParts=built['independentFloorParts'],
                componentLedgerMassKg=built['totalMassKg'], independentFloorMassKg=built['independentFloorMassKg'],
                deltaComponentLedgerKg=built['totalMassKg']-frozen['totalMassKg'], axes=axes,
                excludedFromCarriedBodies=sorted(floor_ids),
                assumptions=['Whole cable-chain moving mass remains a conservative upper bound',
                             'Carried tensors retain each component distribution assumption',
                             'Do not add stock-beam density to aggregate weldment mass in a structural model'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('built', type=Path)
    parser.add_argument('frozen', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = compare(args.built, args.frozen)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for row in result['axes']:
        print(row['axis'], 'moving kg', row['built']['massKg'], 'delta kg', row['deltaMassKg'])


if __name__ == '__main__':
    main()
