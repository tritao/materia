#!/usr/bin/env python3
"""Gravity/zero-speed load screen with the actual CAD dual tool and blanks.

No executed tending cycle, thermal rating or hardware qualification is inferred.
"""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
ARM = ROOT.parent / 'zeroerr-arm'
DIRECTORY = ROOT / 'build/cell-registration'
spec = importlib.util.spec_from_file_location('zeroerr_engineering', ARM/'engineering-study.py')
engineering = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engineering)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(frame, scale=1.):
    x, y, z, w = [frame[k] for k in ('qx', 'qy', 'qz', 'qw')]
    np.testing.assert_allclose(x*x+y*y+z*z+w*w, 1., atol=1e-10)
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    result = np.eye(4)
    result[:3, :3] += 2*w*cross + 2*cross@cross
    result[:3, 3] = [frame[k]*scale for k in 'xyz']
    return result


def aggregate(bodies):
    mass = sum(body[0] for body in bodies)
    center = sum(m*p for m, p, _ in bodies)/mass
    inertia = np.zeros((3, 3))
    for m, p, tensor in bodies:
        delta = p-center
        inertia += tensor + m*(np.dot(delta, delta)*np.eye(3)-np.outer(delta, delta))
    np.testing.assert_allclose(inertia, inertia.T, atol=1e-12)
    if np.linalg.eigvalsh(inertia).min() < -1e-12:
        raise ValueError('Nonphysical aggregate inertia')
    return mass, center, inertia


def placed(body, frame):
    mass, center, tensor = body
    r = frame[:3, :3]
    return mass, r@center+frame[:3, 3], r@tensor@r.T


def main():
    original = '--original-reference' in sys.argv
    unrestricted = '--unrestricted-modified-reference' in sys.argv
    if original and unrestricted:
        raise ValueError('Choose one reference study')
    prefix = 'original-' if original else 'unrestricted-' if unrestricted else ''
    fixture_path = DIRECTORY/'fixtures.json'
    fixtures = json.loads(fixture_path.read_text())
    registration_config = ROOT/'tending-registration-config.json'
    if fixtures['configSha256'] != digest(registration_config):
        raise ValueError('Stale fixture configuration')
    props_path = DIRECTORY/'tool-properties.json'
    properties = json.loads(props_path.read_text())
    if properties['fixtureSha256'] != digest(fixture_path):
        raise ValueError('Stale tool properties')
    current = (dict(layoutPath=str(ARM/'reference-layout.json')) if original else
               json.loads((ARM/'build/straight/current.json').read_text()))
    layout_path = Path(current['layoutPath'])
    layout = json.loads(layout_path.read_text())
    prepared = {} if original else json.loads(Path(current['preparationPath']).read_text())
    for key, filename in ([] if original else [('configSha256', 'straight-link-config.json'), ('generatorSha256', 'straight-link.cpp'),
                          ('preparerSha256', 'prepare-straight.py'), ('baselineLayoutSha256', 'revised-layout.json')]):
        if prepared['inputs'][key] != digest(ARM/filename):
            raise ValueError('Stale prepared arm: '+key)
    geometry_path = ARM/('build/engineering-input.json' if original else 'build/straight/engineering-input.json')
    geometry = json.loads(geometry_path.read_text())
    if geometry['layoutSha256'] != digest(layout_path):
        raise ValueError('Stale arm geometry')
    config_path = ARM/('original-comparison-config.json' if original else 'build/straight/engineering-config.json')
    config = json.loads(config_path.read_text())
    if not original and (config['straightVariant']['baselineConfigSha256'] != digest(ARM/'revised-engineering-config.json') or config['straightVariant']['overrideSha256'] != digest(ARM/'straight-engineering-overrides.json')):
        raise ValueError('Changed arm load assumptions')
    sizing = json.loads((ARM/('build/sizing-report.json' if original else 'build/straight/sizing-report.json')).read_text())
    if sizing['configSha256'] != digest(config_path) or sizing['geometrySha256'] != digest(geometry_path):
        raise ValueError('Stale arm load basis')
    report_path = DIRECTORY/(prefix+'registration-planned.json')
    registration = json.loads(report_path.read_text())
    if registration['fixtureSha256'] != digest(fixture_path) or registration['configSha256'] != digest(registration_config):
        raise ValueError('Stale candidate registration')
    scan = json.loads((DIRECTORY/(prefix+'registration-review.json')).read_text())
    model_id = 'zeroerr-original' if original else 'zeroerr-unrestricted' if unrestricted else 'zeroerr'
    model = next(m for m in registration['models'] if m['model']==model_id)
    baseline = next(m for m in scan['models'] if m['model']==model_id)
    if model['sourceFingerprint'] != baseline['sourceFingerprint']:
        raise ValueError('Changed registered prototype')
    source_path = DIRECTORY/(model_id+'-study-source.mtrg') if original or unrestricted else ARM/'build/straight/zeroerr5-straight80.mtrg'
    if model['sourceFingerprint'] != digest(source_path):
        raise ValueError('Changed prototype artifact')
    candidate = model['candidates'][0]
    poses = [np.array(p['q']) for p in candidate['poses'] if p['q'] is not None]
    for route in candidate['routes']:
        if (route.get('continuous') or {}).get('clear') is True:
            poses += [np.array(q) for q in route['path']]
    if not poses:
        raise ValueError('No evaluated prototype configurations')
    mount = matrix(fixtures['frames']['toolMountZ'], .001)
    inverse_mount = np.linalg.inv(mount)
    stock = properties['stock']
    stock_body = stock['massKg'], np.array(stock['centerLocalMetres']), np.array(stock['inertiaLocalKgM2']).reshape(3, 3)
    tool_to_stock = {c: inverse_mount@matrix(fixtures['frames']['tool'+c], .001)@np.linalg.inv(matrix(fixtures['graspLocal'], .001))
                     for c in ('A', 'B')}
    occurrence = next(o for o in layout['occurrences'] if o['id']=='toolFlange')
    source = np.array(occurrence['matrix']).reshape(3, 4)
    flange = np.eye(4)
    flange[:3, :3] = source[:, :3]@np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    flange[:3, 3] = source[:, 3]/1000
    study = engineering.Study(geometry, config)
    active = next(b[4] for b in study.bodies if b[0]=='tool+payload')
    structure = [b for b in study.bodies if b[0]!='tool+payload']
    rated = np.array([config['catalog'][j['family']]['ratedTorqueNm'][config['catalog'][j['family']]['ratios'].index(j['nominalRatio'])] for j in config['jointCandidates']])
    alpha = np.deg2rad(config['application']['jointAccelerationDegS2'])
    factor = config['application']['massSensitivityFactor']
    results = []
    for state in properties['states']:
        bodies = []
        for part in state['parts']:
            body = part['massKg'], np.array(part['centerLocalMetres']), np.array(part['inertiaLocalKgM2']).reshape(3, 3)
            bodies.append(placed(body, inverse_mount@matrix(part['frameMetres'])))
        tool_mass, tool_center, _ = aggregate(bodies)
        if state['holding']=='empty':
            np.testing.assert_allclose(tool_mass, fixtures['toolMassEstimate']['massKg'], atol=1e-8)
            np.testing.assert_allclose(tool_center, np.array(fixtures['toolMassEstimate']['centerOfMassAtMountZMm'])/1000, atol=1e-7)
        for c in ('A', 'B'):
            if c in state['holding']:
                bodies.append(placed(stock_body, tool_to_stock[c]))
        mass, center, inertia = aggregate(bodies)
        m, p, tensor = placed((mass, center, inertia), flange)
        study.bodies = structure+[('tool+payload', m, p, tensor, active)]
        nominal, heavy, envelope = np.zeros(6), np.zeros(6), np.zeros(6)
        for index, q in enumerate(poses):
            gravity, _, _, potential = study.terms(q)
            g_heavy, matrix_heavy, _, _ = study.terms(q, factor)
            nominal = np.maximum(nominal, np.abs(gravity))
            heavy = np.maximum(heavy, np.abs(g_heavy))
            envelope = np.maximum(envelope, (np.abs(g_heavy)+np.abs(matrix_heavy).sum(axis=1)*alpha)*config['application']['designFactor'])
            if index < 3:
                eps = 1e-6
                gradient = np.array([(study.terms(q+np.eye(6)[j]*eps)[3]-study.terms(q-np.eye(6)[j]*eps)[3])/(2*eps) for j in range(6)])
                np.testing.assert_allclose(gravity, gradient, atol=1e-5, rtol=1e-6)
        results.append(dict(holding=state['holding'],toolMassKg=tool_mass,flangeLoadKg=mass,centerOfMassAtFlangeMetres=center.tolist(),
                            inertiaAtCenterKgM2=inertia.tolist(),maxNominalGravityNm=nominal.tolist(),maxHeavyGravityNm=heavy.tolist(),
                            maxHeavyAccelerationEnvelopeWithDesignFactorNm=envelope.tolist(),ratioToCatalogRated=(envelope/rated).tolist()))
    result = dict(status='preliminary_registered_configuration_load_screen',configurations=len(poses),baseWorldMetres=candidate['baseWorldMetres'],
                  stockMassKg=stock['massKg'],states=results,
                  inputs={str(p.relative_to(ROOT.parent)):digest(p) for p in (fixture_path, props_path, report_path, geometry_path, config_path, layout_path, source_path)},
                  analysisSha256=digest(Path(__file__)),
                  limitations=['Configurations are evaluated poses/path vertices, not an executable or thermally qualified cycle.',
                               'Arm module internals retain the catalog-mass/exterior-volume inertia surrogate.',
                               'Tool component declared/CAD mass and inertia are estimates, not measured hardware.',
                               'Zero-speed acceleration envelope; no velocity coupling, friction, efficiency, cable drag or fatigue.',
                               'Generic tool adapter fit, grasp force, physical travel and full tending clearance remain unqualified.',
                               'The separate full-workspace 5 kg benchmark and unresolved shoulder decision still apply.'])
    (DIRECTORY/(prefix+'registered-load-report.json')).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'configurations':len(poses),'toolMassKg':results[0]['toolMassKg'],'stockMassKg':stock['massKg'],
                      'doubleHeldLoadKg':results[-1]['flangeLoadKg'],'worstShoulderRatio':max(r['ratioToCatalogRated'][1] for r in results)},indent=2))


if __name__ == '__main__':
    main()
