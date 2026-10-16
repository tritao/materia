#!/usr/bin/env python3
"""Validate and compare unchanged supplier CAD against our modified arm."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
REF = ROOT.parent / 'reference-arms'
CELL = REF / 'build/cell-registration'
BUILD = ROOT / 'build/original-evaluation'
inputs = {}
spec = importlib.util.spec_from_file_location('engineering', ROOT / 'engineering-study.py')
engineering = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engineering)


def digest(path):
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    inputs[str(path.resolve())] = value
    return value


def read(path):
    digest(path)
    return json.loads(path.read_text())


def require(ok, message):
    if not ok:
        raise ValueError(message)


def filtered_loads(hulls):
    digest(ROOT / 'engineering-study.py')
    original = read(ROOT / 'original-comparison-config.json')
    modified = read(ROOT / 'build/straight/engineering-config.json')
    require(original['application'] == modified['application'], 'Different comparison load/motion assumptions')
    require(original['missionStudy'] == modified['missionStudy'], 'Different synthetic cycles')
    require(original['benchmarks'] == modified['benchmarks'], 'Different benchmark definitions')
    records = []
    require({r['model'] for r in hulls['models']} == {'zeroerr-original', 'zeroerr'}, 'Missing comparison model')
    for row in hulls['models']:
        is_original = row['model'] == 'zeroerr-original'
        config = original if is_original else modified
        geometry_path = ROOT / ('build/engineering-input.json' if is_original else 'build/straight/engineering-input.json')
        source_path = ROOT / ('build/zeroerr5.mtrg' if is_original else 'build/straight/zeroerr5-straight80.mtrg')
        require(row['sourceFingerprint'] == digest(source_path), 'Stale hull-comparison source')
        geometry = read(geometry_path)
        layout_path = ROOT / 'reference-layout.json' if is_original else Path(read(ROOT / 'build/straight/current.json')['layoutPath'])
        require(geometry['layoutSha256'] == digest(layout_path), 'Stale arm geometry')
        study = engineering.Study(geometry, config)
        require(len(row['loadObservations']) == 270, 'Incomplete load-pose collision filter')
        poses = [np.array(p['joints']) for p in row['loadObservations'] if p['clear']]
        require(bool(poses), 'No hull-clear load poses')
        scenarios = []
        rated = np.array([config['catalog'][j['family']]['ratedTorqueNm'][config['catalog'][j['family']]['ratios'].index(j['nominalRatio'])]
                          for j in config['jointCandidates']])
        for factor in (1., config['application']['massSensitivityFactor']):
            gravity, envelope = np.zeros(6), np.zeros(6)
            for q in poses:
                g, matrix, _, _ = study.terms(q, factor)
                gravity = np.maximum(gravity, np.abs(g))
                envelope = np.maximum(envelope, (np.abs(g) + np.abs(matrix).sum(axis=1) *
                    np.deg2rad(config['application']['jointAccelerationDegS2'])) * config['application']['designFactor'])
            scenarios.append(dict(structuralMassFactor=factor, gravityNm=gravity.tolist(),
                                  accelerationEnvelopeWithFactorNm=envelope.tolist(),
                                  ratioToAssumedRated=(envelope/rated).tolist()))
        records.append(dict(model=row['model'], estimatedArmMassKg=sum(b[1] for b in study.bodies if b[0] != 'tool+payload'),
                            hullClearLoadPoses=len(poses), ratedTorqueNm=rated.tolist(), studies=scenarios))
    result = dict(status='matched_bare_arm_hull_filtered_load_screen', models=records, assumptions=original['application'],
                  scope='Same 270 joint poses; only enclosing-hull-clear bare-arm poses retained. Same point flange load. '
                        'No physical tool envelope, fixtures, velocity terms, thermal/RMS or hardware qualification. '
                        'Nominal-100 ratios and catalog/exterior-volume mass split are assumptions, not confirmed vendor ordering codes.',
                  producerSha256=digest(Path(__file__)), inputs=inputs.copy())
    (BUILD / 'filtered-load-comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def base_key(candidate):
    return tuple(round(candidate['baseWorldMetres'][axis], 8) for axis in 'xyz')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--loads-only', action='store_true')
    parser.add_argument('--prepare-load-probes', action='store_true')
    args = parser.parse_args()
    if args.prepare_load_probes:
        config = read(ROOT / 'original-comparison-config.json')
        app = config['application']
        lo, hi = np.deg2rad(app['syntheticJointRangeDeg'])
        poses = [np.zeros(6), np.array([.2, -.8, 1., .1, -.6, .2])]
        poses += [np.eye(6)[i]*angle for i in range(6) for angle in (lo, hi)]
        poses += [np.array([lo+(hi-lo)*engineering.halton(n,b) for b in (2,3,5,7,11,13)])
                  for n in range(1,app['sampleCount']+1)]
        BUILD.mkdir(parents=True, exist_ok=True)
        path = BUILD / 'load-probes.json'
        values = [q.tolist() for q in poses]
        if not path.exists() or json.loads(path.read_text()) != values:
            path.write_text(json.dumps(values, indent=2)+'\n')
        print(path)
        return
    hulls = read(BUILD / 'hull-comparison.json')
    require(hulls['producerSha256'] == digest(REF / 'OriginalArmReview.hx'), 'Stale hull checker')
    require(hulls['probeSha256'] == digest(ROOT / 'build/straight/required-probes.json'), 'Changed diagnostic probes')
    require(hulls['loadProbeSha256'] == digest(BUILD / 'load-probes.json'), 'Changed load probes')
    filtered = filtered_loads(hulls)
    if args.loads_only:
        print(json.dumps(filtered['models'], indent=2))
        return
    provenance = read(ROOT / 'build/zeroerr5.provenance.json')
    require(provenance['artifactSha256'] == digest(ROOT / 'build/zeroerr5.mtrg') and
            provenance['layoutSha256'] == digest(ROOT / 'reference-layout.json') and
            provenance['exporterSha256'] == digest(ROOT / 'ZeroErrArmPreview.hx') and
            provenance['planningWindowSha256'] is None, 'Stale/modified original project')
    layout = read(ROOT / 'reference-layout.json')
    for asset in layout['assets']:
        require(asset['sha256'] == digest(ROOT / 'build/vendor' / (asset['id']+'.STEP')), 'Changed original supplier geometry')
    exact = read(BUILD / 'motion-report.json')
    require(exact['uniqueRequiredPoses'] == exact['uniqueObservedPoses'] == 60 and not exact['missingPosesRad'], 'Incomplete exact CAD review')
    require(exact['layoutSha256'] == provenance['layoutSha256'] and
            exact['engineeringInputSha256'] == digest(ROOT / 'build/engineering-input.json'), 'Stale exact CAD geometry')
    for path, expected in exact['inputSha256'].items():
        require(digest(Path(path)) == expected, 'Changed original CAD checkpoint')
    curve = read(ROOT / 'build/continuous-clearance.json')
    require(curve['artifactSha256'] == provenance['artifactSha256'] and curve['status'] == 'clear_within_stated_hull_policy' and
            curve['configSha256'] == digest(ROOT / 'original-comparison-config.json'), 'Stale/failed continuous cycle check')
    merged = read(CELL / 'original-registration-planned.json')
    require(merged['mergeProducerSha256'] == digest(REF / 'plan-original-candidates.py'), 'Stale candidate merger')
    require(merged['candidateScanSha256'] == digest(CELL / 'original-registration-review.json'), 'Changed candidate scan')
    for path, expected in merged['independentSearchEvidenceSha256'].items():
        require(digest(Path(path)) == expected, 'Changed independent candidate evidence')
    results = []
    fixture_hash = digest(CELL / 'fixtures.json')
    config_hash = digest(REF / 'tending-registration-config.json')
    for prefix, model_id in [('original-', 'zeroerr-original'), ('unrestricted-', 'zeroerr-unrestricted'), ('', 'zeroerr')]:
        report = read(CELL / (prefix+'registration-planned.json'))
        require(report['fixtureSha256'] == fixture_hash and report['configSha256'] == config_hash, 'Changed fixture/task assumptions')
        model = next(m for m in report['models'] if m['model'] == model_id)
        source = CELL / (model_id+'-study-source.mtrg') if prefix else ROOT / 'build/straight/zeroerr5-straight80.mtrg'
        require(model['sourceFingerprint'] == digest(source), 'Stale studied arm')
        require(len(model['examinedCandidates']) == 9 and len(model['candidates']) == 1, 'Incomplete base search')
        candidate = model['candidates'][0]
        require(len(candidate['poses']) == 82 and len(candidate['routes']) == 23, 'Incomplete task coverage')
        good = [r for r in candidate['routes'] if (r.get('continuous') or {}).get('clear')]
        require(len(good) == max(c['continuousRoutes'] for c in model['examinedCandidates']), 'Incorrect selected candidate')
        for route in good:
            require(route['path'] and len(route['path']) >= 2 and route['continuous']['error'] is None, 'Clear route lacks valid proof/path')
        load = read(CELL / (prefix+'registered-load-report.json'))
        require(load['analysisSha256'] == digest(REF / 'registered-load-study.py'), 'Stale registered load producer')
        for path, expected in load['inputs'].items():
            require(digest(ROOT.parent / path) == expected, 'Stale registered load input')
        reasons = {}
        for pose in candidate['poses']:
            reasons[pose['reason']] = reasons.get(pose['reason'], 0)+1
        results.append(dict(model=model_id, baseWorldMetres=candidate['baseWorldMetres'], clearTargets=candidate['clearPoses'],
                            continuousTransfers=len(good), loadConfigurations=load['configurations'], reasons=reasons,
                            nominalShoulderGravityNm=load['states'][-1]['maxNominalGravityNm'][1],
                            conservativeShoulderNm=load['states'][-1]['maxHeavyAccelerationEnvelopeWithDesignFactorNm'][1]))
    numeric = read(CELL / 'original-numeric-registration-review.json')
    analytic = read(CELL / 'original-registration-review.json')
    require(numeric['fixtureSha256'] == analytic['fixtureSha256'] == fixture_hash and
            numeric['configSha256'] == analytic['configSha256'] == config_hash, 'Mismatched solver control')
    controls = numeric['models'][0]
    require(controls['solverMode'] == 'numeric_only_control' and len(controls['candidates']) == 9 and
            controls['sourceFingerprint'] == analytic['models'][0]['sourceFingerprint'], 'Invalid numerical-only control')
    by_base = {base_key(c): c['clearPoses'] for c in controls['candidates']}
    control_rows = [dict(baseWorldMetres=c['baseWorldMetres'], analyticClearTargets=c['clearPoses'], numericClearTargets=by_base[base_key(c)])
                    for c in analytic['models'][0]['candidates']]
    app = read(REF / 'build/application-validation.json')
    require(app['originalReferenceChecked'] and app['originalArtifactSha256'] == provenance['artifactSha256'] and
            app['originalCellArtifactSha256'] == digest(CELL / 'zeroerr-original-cell.mtrg'), 'Stale/missing original application check')
    result = dict(status='original_reference_evaluated_not_hardware_qualified',
                  originalExact=dict(clear=exact['clearPoses'], colliding=exact['collisionPoses'], indeterminate=exact['indeterminatePoses']),
                  equalPoseHullResults=[dict(model=m['model'], clear=m['clearPoses'], total=m['poses'], ikBackend=m['ikBackend']) for m in hulls['models']],
                  filteredLoads=filtered['models'], tending=results, numericControl=control_rows,
                  originalContinuousCycle=curve['status'], originalPlanningWindowDeg=[-180, 180],
                  physicalQualification=False, completeTendingCycle=False,
                  producerSha256=digest(Path(__file__)), inputs=inputs)
    (BUILD / 'evaluation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('inputs', 'filteredLoads', 'numericControl')}, indent=2))


if __name__ == '__main__':
    main()
