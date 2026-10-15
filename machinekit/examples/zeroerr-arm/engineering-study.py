#!/usr/bin/env python3
"""Reproducible preliminary load screen; not drive, brake or payload qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
G = 9.80665


def rotation(axis, angle):
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def halton(n, base):
    value, factor = 0., 1.
    while n:
        n, digit = divmod(n, base)
        factor /= base
        value += digit*factor
    return value


class Study:
    def __init__(self, geometry, config):
        self.config = config
        self.app = config['application']
        self.origins = np.array([a['originMm'] for a in geometry['axes']])/1000
        self.axes = np.array([a['axis'] for a in geometry['axes']])
        parent = {j['child']: j for j in geometry['joints']}

        def ancestors(member):
            active, seen = [], set()
            while member in parent:
                if member in seen:
                    raise ValueError('Cycle in assembly tree')
                seen.add(member)
                joint = parent[member]
                if joint['type'] == 'revolute':
                    active.append(int(joint['id'][1:])-1)
                member = joint['parent']
            if member != 'base':
                raise ValueError(f'Disconnected member {member}')
            return sorted(active)

        totals = [sum(s['volumeMm3'] for s in geometry['solids']
                      if s['id'].startswith(f'joint{i+1}-')) for i in range(6)]
        self.bodies = []
        for s in geometry['solids']:
            v = s['volumeMm3']
            if not np.isfinite(v) or v <= 0:
                raise ValueError('Invalid CAD volume')
            # Catalog mass replaces all exterior CAD mass for each complete module.
            # Distribution among housing/flange solids is only a uniform-density surrogate.
            if s['id'].startswith('joint'):
                j = int(s['id'][5])-1
                family = config['jointCandidates'][j]['family']
                mass = v/totals[j]*config['catalog'][family]['brakeMassKg']
            else:
                # Assumed 7075 density for links; other exterior fittings approximated
                # as aluminium. No claim of known fastener/connector material.
                mass = v*(2.81e-6 if s['id'] in ('upperArm', 'forearm') else 2.70e-6)
            inertia = np.array(s['inertiaMm5'])*(mass/v)*1e-6
            self.bodies.append((s['id'], mass, np.array(s['centroidMm'])/1000,
                                inertia, ancestors(s['id'])))
        tip = np.array(geometry['toolFaceMm'])/1000
        tip += np.array(geometry['toolNormal'])*self.app['combinedCogAlongToolNormalMm']/1000
        # Payload and tool treated as point masses: intrinsic inertia remains unknown.
        self.bodies.append(('tool+payload', self.app['toolKg']+self.app['payloadKg'],
                            tip, np.zeros((3, 3)), ancestors('toolFlange')))
        self.reflected = np.array([
            config['catalog'][j['family']]['motorRotorInertiaGmm2']*1e-9*(j['nominalRatio']+1)**2
            for j in config['jointCandidates']])

    def terms(self, q, mass_factor=1.):
        rotations = [rotation(a, angle) for a, angle in zip(self.axes, q)]

        def transform(point, active):
            r = np.eye(3)
            p = point.copy()
            for i in reversed(active):
                p = self.origins[i]+rotations[i]@(p-self.origins[i])
                r = rotations[i]@r
            return p, r

        current_axes, current_origins = [], []
        for i in range(6):
            o, r = transform(self.origins[i], range(i))
            current_axes.append(r@self.axes[i])
            current_origins.append(o)
        matrix, gravity, payload_gravity = np.zeros((6, 6)), np.zeros(6), np.zeros(6)
        potential = 0.
        for name, mass, point, inertia, active in self.bodies:
            factor = 1. if name == 'tool+payload' else mass_factor
            mass, inertia = mass*factor, inertia*factor
            p, r = transform(point, active)
            jv, jw = np.zeros((3, 6)), np.zeros((3, 6))
            for i in active:
                jw[:, i] = current_axes[i]
                jv[:, i] = np.cross(current_axes[i], p-current_origins[i])
            matrix += mass*jv.T@jv + jw.T@r@inertia@r.T@jw
            torque = mass*G*jv[2, :]
            gravity += torque
            potential += mass*G*p[2]
            if name == 'tool+payload':
                payload_gravity += torque
        matrix += np.diag(self.reflected)
        return gravity, matrix, payload_gravity, potential


def run(config_path):
    config = json.loads(config_path.read_text())
    raw = (HERE/'build/engineering-input.json').read_bytes()
    geometry = json.loads(raw)
    layout_hash = hashlib.sha256((HERE/'reference-layout.json').read_bytes()).hexdigest()
    if geometry['layoutSha256'] != layout_hash:
        raise ValueError('Stale engineering geometry; export --engineering again')
    study = Study(geometry, config)
    app = config['application']
    if (any(not np.isfinite(app[k]) or app[k] < 0 for k in
            ('payloadKg', 'toolKg', 'combinedCogAlongToolNormalMm', 'jointSpeedDegS', 'jointAccelerationDegS2'))
            or app['designFactor'] < 1 or app['massSensitivityFactor'] < 1 or app['sampleCount'] < 1):
        raise ValueError('Invalid study assumptions')
    lo, hi = np.deg2rad(app['syntheticJointRangeDeg'])
    poses = [np.zeros(6), np.array([.2, -.8, 1., .1, -.6, .2])]
    poses += [np.eye(6)[i]*angle for i in range(6) for angle in (lo, hi)]
    poses += [np.array([lo+(hi-lo)*halton(n, b) for b in (2, 3, 5, 7, 11, 13)])
              for n in range(1, app['sampleCount']+1)]
    # Mechanics checks: gravity equals potential gradient; mass matrix is symmetric PSD.
    probe = np.array([.17, -.63, .48, .29, -.37, .23])
    g, m, _, _ = study.terms(probe)
    gradient = []
    for i in range(6):
        dq = np.eye(6)[i]*1e-6
        gradient.append((study.terms(probe+dq)[3]-study.terms(probe-dq)[3])/2e-6)
    np.testing.assert_allclose(g, gradient, atol=1e-6, rtol=1e-6)
    np.testing.assert_allclose(m, m.T, atol=1e-10)
    if np.linalg.eigvalsh(m).min() < -1e-10:
        raise ValueError('Mass matrix is not positive semidefinite')
    # Closed-form pendulum fixture exercises the actual solver in SI units.
    pendulum = Study.__new__(Study)
    pendulum.origins = np.zeros((6, 3))
    pendulum.axes = np.tile([0., 1., 0.], (6, 1))
    pendulum.reflected = np.zeros(6)
    pendulum.bodies = [('tool+payload', 6., np.array([.15, 0., 0.]), np.zeros((3, 3)), [1])]
    pg, pm, _, _ = pendulum.terms(np.zeros(6))
    np.testing.assert_allclose(pg[1], -8.825985)
    np.testing.assert_allclose(pm[1, 1], .135)
    np.testing.assert_allclose(pendulum.terms(np.eye(6)[1]*np.pi/2)[0], 0., atol=1e-12)
    np.testing.assert_allclose(g[0], 0., atol=1e-10)  # vertical base yaw has no gravity moment
    reports = []
    for mass_factor in (1., app['massSensitivityFactor']):
        peaks, at = np.zeros((3, 6)), [[None]*6 for _ in range(3)]
        for q in poses:
            gravity, inertia, payload, _ = study.terms(q, mass_factor)
            # All combinations of bounded simultaneous accelerations, at zero speed.
            # Velocity-dependent, friction and flexible effects are explicitly omitted.
            values = [np.abs(gravity), np.abs(payload),
                      np.abs(gravity)+np.abs(inertia).sum(axis=1)*np.deg2rad(app['jointAccelerationDegS2'])]
            for category, value in enumerate(values):
                for j in range(6):
                    if value[j] >= peaks[category, j]:
                        peaks[category, j] = value[j]
                        at[category][j] = q.tolist()
        rows = []
        for i, choice in enumerate(config['jointCandidates']):
            cat = config['catalog'][choice['family']]
            idx = cat['ratios'].index(choice['nominalRatio'])
            rated, peak = cat['ratedTorqueNm'][idx], cat['startStopPeakNm'][idx]
            rows.append(dict(joint=choice['joint'], family=choice['family'], ratio=choice['nominalRatio'],
                sampledGravityNm=peaks[0, i], sampledPayloadOnlyGravityNm=peaks[1, i],
                payloadOnlyWithFactorOverRated=peaks[1, i]*app['designFactor']/rated,
                sampledZeroSpeedAccelerationEnvelopeNm=peaks[2, i], ratedTorqueNm=rated,
                startStopPeakNm=peak, staticWithFactorOverRated=peaks[0, i]*app['designFactor']/rated,
                accelerationWithFactorOverStartStop=peaks[2, i]*app['designFactor']/peak,
                speedOnlyWithinCatalog=app['jointSpeedDegS']/6 <= cat['ratedLoadSpeedRpm48V'][idx],
                catalogSpeedReserveDegS=cat['ratedLoadSpeedRpm48V'][idx]*6-app['jointSpeedDegS'],
                gravityWitnessRad=at[0][i], accelerationWitnessRad=at[2][i]))
        reports.append(dict(structuralMassFactor=mass_factor, joints=rows))
    result = dict(status='screening_only_hardware_configuration_not_released',
        geometrySha256=hashlib.sha256(raw).hexdigest(), layoutSha256=layout_hash,
        configSha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        assumptions=app, samplePoses=len(poses),
        nominalArmMassKg=sum(b[1] for b in study.bodies if b[0] != 'tool+payload'),
        scope='Sampled poses may self-collide. Not a collision-filtered trajectory or duty-cycle qualification. '
              'Module mass distribution is an exterior-volume surrogate; payload intrinsic inertia, '
              'Coriolis/centrifugal terms, friction, efficiency, cable drag and thermal limits omitted.',
        benchmarks=config['benchmarks'],
        ratioSpeedComparison={family: [dict(nominalRatio=r, ratedTorqueNm=cat['ratedTorqueNm'][i],
            startStopPeakNm=cat['startStopPeakNm'][i], speedDegS=cat['ratedLoadSpeedRpm48V'][i]*6,
            meetsSpeedTarget=cat['ratedLoadSpeedRpm48V'][i]*6 >= app['jointSpeedDegS'])
            for i, r in enumerate(cat['ratios'])] for family, cat in config['catalog'].items()},
        studies=reports, mechanicsChecks='potential gradient, mass symmetry/PSD, analytical point-load moment passed')
    out = HERE/'build/sizing-report.json'
    out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f'Screened {len(poses)} poses; estimated arm mass {result["nominalArmMassKg"]:.3f} kg')
    print('Joint | gravity Nm | payload-only Nm | zero-speed accel Nm | static*f/rated')
    for row in reports[0]['joints']:
        print(f'{row["joint"]:5} | {row["sampledGravityNm"]:10.2f} | {row["sampledPayloadOnlyGravityNm"]:15.2f} | '
              f'{row["sampledZeroSpeedAccelerationEnvelopeNm"]:19.2f} | {row["staticWithFactorOverRated"]:14.2f}')
    print(out)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'engineering-config.json')
    run(parser.parse_args().config)
