#!/usr/bin/env python3
"""Check current generated artifacts, provenance and review coverage.

Run native checks and application integration as documented before this audit.
This validates their artifact consistency, not physical hardware qualification.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD = ROOT/'build'
ARM = ROOT.parent/'zeroerr-arm'
checked = {}


def digest(path):
    if not path.is_absolute():
        path = ROOT.parents[2]/path
    path = path.resolve()
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    checked[str(path.relative_to(ROOT.parents[2]))] = value
    return value


def read(path):
    digest(path)
    return json.loads(path.read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    lock = read(ROOT/'source-lock.json')
    models = lock['models']
    require(len(models)==22, 'Expected the reviewed 22-model library')
    preparation_report = read(BUILD/'preparation-report.json')
    require({r['model'] for r in preparation_report if r['status']=='prepared'}==set(models), 'Incomplete range preparation')
    rejected = [r for r in preparation_report if r['status']=='failed']
    require({r['model'] for r in rejected}=={'FR3WML','FR3WMS'} and
            all('undeclared links' in r['reason'] and 'tool_Link' in r['reason'] for r in rejected), 'Unrecorded model source gaps')
    round_trips = 0
    for name in models:
        directory = BUILD/name
        manifest = read(directory/'manifest.json')
        validation = read(directory/'validation.json')
        require(validation['model']==name and validation['roundTrips']==39 and len(manifest['probes'])==39,
                'Incomplete FK/IK coverage: '+name)
        require(validation['hardwareConnected'] is False, 'Hardware boundary changed')
        require(validation['sceneBytes']==(directory/(name+'.scene')).stat().st_size, 'Changed saved reference scene')
        require(manifest['source']['revision']==models[name]['revision'], 'Changed official source revision')
        require(digest(directory/'robot.urdf')==manifest['urdfSha256'], 'Changed URDF')
        for link in manifest['links']:
            for mesh in link['visuals']+link['collisions']:
                for suffix, expected in mesh['sha256'].items():
                    if expected is not None:
                        require(digest(directory/(mesh['stem']+'.'+suffix))==expected, 'Changed mesh stream')
        round_trips += validation['roundTrips']
    for archive in read(ROOT/'cad-sources.json')['archives']:
        require(digest(BUILD/'cad'/(archive['id']+'.zip'))==archive['sha256'], 'Changed official CAD archive')
    comparison = read(BUILD/'comparison/validation.json')
    require(comparison['status']=='passed' and comparison['movingJoints']==18 and comparison['occurrences']==137,
            'Incomplete combined scene checks')
    for name in ('zeroerr','ur5e','fairino5_v6'):
        task = read(BUILD/'comparison'/(name+'-task.json'))
        require(len(task['samples'])==21 and not task['collisionViolations'] and task['maxPositionError']<=1e-6 and
                task['maxOrientationError']<=1e-6 and task['maxJointJump']<.1, 'Failed common tool task')
    for mounting in ('floor','wall','ceiling'):
        digest(BUILD/'comparison'/(mounting+'.mtrg'))
    app = read(BUILD/'application-validation.json')
    require(set(app['comparisonMountings'])=={'floor','wall','ceiling'} and
            set(app['registeredModels'])=={'zeroerr','ur5e','fairino5_v6'}, 'Incomplete application imports')
    require(len(app['robots'])==2 and all(r['jointMotion']>1e-4 for r in app['robots']), 'No observed reference playback')
    current = read(ARM/'build/straight/current.json')
    layout_path = Path(current['layoutPath'])
    preparation_path = Path(current['preparationPath'])
    layout = read(layout_path)
    preparation = read(preparation_path)
    for key, file in [('configSha256','straight-link-config.json'),('baselineLayoutSha256','revised-layout.json'),
                      ('generatorSha256','straight-link.cpp'),('preparerSha256','prepare-straight.py')]:
        require(preparation['inputs'][key]==digest(ARM/file), 'Changed straight design input')
    for relative, expected in preparation['files'].items():
        require(digest(preparation_path.parent/relative)==expected, 'Changed prepared part')
    require(layout['straightForearm']['config']['tubeOuterDiameterMm']==70, 'Unexpected prototype dimensions')
    audit = read(ARM/'build/straight/exported-parts-audit.json')
    require(audit['preparationSha256']==digest(preparation_path) and audit['layoutSha256']==digest(layout_path), 'Stale CAD audit')
    require(audit['importedValidSolids']==31 and audit['joinedStructuralSolidValid'] and
            audit['testedAxialDuctDiameterMm']==41.998 and audit['ductMaterialOverlapMm3']==0 and
            audit['coverMaterialOverlapMm3']==0 and audit['indeterminateQueries']==0, 'Failed exported-part audit')
    require(audit['auditSourceSha256']==digest(ARM/'audit-straight.cpp'), 'Changed independent CAD audit')
    structure = read(ARM/'build/straight/structure-report.json')
    require(structure['layoutSha256']==digest(layout_path) and not structure['issues'] and
            structure['analysisScriptSha256']==digest(ARM/'structure-study.py'), 'Stale/failed structural screen')
    require(structure['assumptionSha256']==digest(ARM/'straight-engineering-overrides.json'), 'Changed structural assumptions')
    sizing = read(ARM/'build/straight/sizing-report.json')
    require(sizing['layoutSha256']==digest(layout_path) and sizing['geometrySha256']==digest(ARM/'build/straight/engineering-input.json') and
            sizing['configSha256']==digest(ARM/'build/straight/engineering-config.json'), 'Stale load study')
    motion = read(ARM/'build/straight/motion-report.json')
    require(motion['uniqueRequiredPoses']==60 and motion['uniqueObservedPoses']==60 and not motion['missingPosesRad'] and
            motion['clearPoses']>=55 and motion['collisionPoses']<=5 and motion['indeterminatePoses']<=1, 'Incomplete/regressed motion review')
    observations = motion['observations']
    require(len(observations)==60 and motion['clearPoses']==sum(not r['overlaps'] and not r['indeterminatePairs'] for r in observations) and
            motion['collisionPoses']==sum(bool(r['overlaps']) for r in observations) and
            motion['indeterminatePoses']==sum(bool(r['indeterminatePairs']) for r in observations), 'Incorrect exact CAD counts')
    require(motion['layoutSha256']==digest(layout_path), 'Stale motion geometry')
    for path, expected in motion['inputSha256'].items():
        require(digest(Path(path))==expected, 'Changed exact motion observation')
    curve = read(ARM/'build/straight/continuous-clearance.json')
    require(curve['status']=='clear_within_stated_hull_policy' and curve['queries']>0 and
            curve['artifactSha256']==digest(ARM/'build/straight/zeroerr5-straight80.mtrg'), 'Stale/failed synthetic continuous review')
    boundary = read(ARM/'build/straight/clearance-improvement.json')
    window = read(ARM/'straight-planning-window.json')
    require(boundary['layoutSha256']==digest(layout_path) and window['layoutSha256']==digest(layout_path), 'Stale clearance/window geometry')
    require(window['limitsDeg']==boundary['proposedPlanningWindowDeg'] and window['requiresPoseAndContinuousCollisionChecks'] and
            not window['hardwareQualified'], 'Simulation restriction does not match its conditional review')
    require(boundary['analysisSha256']==digest(ARM/'review-clearance-boundaries.py'), 'Changed clearance analysis')
    for name, expected in boundary['reportsSha256'].items():
        path = ARM/'build/straight'/('clearance-boundaries-'+name+'.json')
        report = read(path)
        require(digest(path)==expected and report['producerSha256']==digest(ROOT/'StraightBoundaryReview.hx') and
                report['configSha256']==digest(ARM/'clearance-review-config.json') and len(report['scans'])==11, 'Stale/missing boundary slice')
    require(window['reviewConfigSha256']==digest(ARM/'clearance-review-config.json'), 'Changed slice basis')
    equivalence = read(ARM/'build/straight/cover-geometry-equivalence.json')
    require(equivalence['status']=='byte_identical_geometry_except_two_covers' and equivalence['layoutSha256']==digest(layout_path) and
            equivalence['producerSha256']==digest(ARM/'review-cover-regression.py'), 'Stale/incomplete geometry reuse proof')
    require(motion['coverGeometryEquivalenceSha256']==digest(ARM/'build/straight/cover-geometry-equivalence.json'), 'Motion report lacks its reuse proof')
    for name, row in equivalence['unchangedFiles'].items():
        require(row['currentSha256']==digest(Path(current['vendorDirectory'])/name), 'Changed reused geometry')
    for path, expected in motion['freshCoverCheckpointSha256'].items():
        path = Path(path)
        metadata = read(Path(str(path)+'.provenance.json'))
        require(digest(path)==expected and metadata['status']=='complete' and metadata['layoutSha256']==digest(layout_path) and
                metadata['producerSha256']==digest(ARM/'ZeroErrArmPreview.hx'), 'Stale/incomplete fresh cover observation')
    restriction = read(ARM/'build/straight/planning-window-review.json')
    require(not restriction['unsafeRetainedPoses'] and restriction['retainedPoses']==54 and
            restriction['excludedPoses']==6 and restriction['motionReportSha256']==digest(ARM/'build/straight/motion-report.json') and
            restriction['planningWindowSha256']==digest(ARM/'straight-planning-window.json'), 'Failed/stale planning-window observation check')
    provenance = read(ARM/'build/straight/zeroerr5-straight80.provenance.json')
    require(provenance['planningWindowSha256']==digest(ARM/'straight-planning-window.json') and
            provenance['artifactSha256']==digest(ARM/'build/straight/zeroerr5-straight80.mtrg') and
            provenance['exporterSha256']==digest(ARM/'ZeroErrArmPreview.hx'), 'Export lacks current planning restrictions/provenance')
    require(app['prototypeArtifactSha256']==provenance['artifactSha256'] and
            app['prototypePlanningWindowSha256']==provenance['planningWindowSha256'], 'Stale application integration report')
    pack = read(ARM/'build/straight/prototype-pack.json')
    require(pack['layoutSha256']==digest(layout_path) and pack['exportedPartAuditSha256']==digest(ARM/'build/straight/exported-parts-audit.json') and
            pack['structureReportSha256']==digest(ARM/'build/straight/structure-report.json'), 'Stale prototype pack')
    for file in pack['files']:
        digest(ARM/'build/straight'/file)
    bench = read(ARM/'bench-request.json')
    require(bench['status']=='draft_only_not_sent_not_ordered' and bench['quantity']==1, 'Bench scope changed')
    cell = BUILD/'cell-registration'
    fixture = read(cell/'fixtures.json')
    require(fixture['configSha256']==digest(ROOT/'tending-registration-config.json'), 'Stale candidate fixture configuration')
    for file, key in [('mill.mtrg','millArtifactSha256'),('tool.mtrg','toolArtifactSha256'),('trays.mtrg','trayArtifactSha256')]:
        require(digest(cell/file)==fixture[key], 'Changed candidate geometry')
    for state in fixture['toolStates']:
        require(digest(cell/state['file'])==state['sha256'], 'Changed physical jaw state')
    planned = read(cell/'registration-planned.json')
    require(planned['fixtureSha256']==digest(cell/'fixtures.json') and planned['configSha256']==digest(ROOT/'tending-registration-config.json'), 'Stale transfer review')
    expected_routes = {}
    for model in planned['models']:
        name = model['model']
        require(len(model['candidates'])==1, 'Expected one selected candidate per arm')
        candidate = model['candidates'][0]
        require(len(candidate['routes'])==23, 'Missing production transfer request')
        good = [r for r in candidate['routes'] if r.get('continuous') and r['continuous']['clear']]
        expected_routes[name] = len(good)
        if name!='zeroerr':
            require(len(good)=={'ur5e':10,'fairino5_v6':18}[name], 'Changed unchanged-reference route coverage')
        else:
            examined = model['examinedCandidates']
            require(len(examined)==9 and len(good)==max(c['continuousRoutes'] for c in examined),
                    'Incomplete base search or incorrect selected candidate')
        require(model['sourceFingerprint']==digest(ARM/'build/straight/zeroerr5-straight80.mtrg') if name=='zeroerr' else True,
                'Stale prototype tending review')
        for route in good:
            require(route['path'] is not None and len(route['path'])>=2 and route['continuous']['error'] is None,
                    'Clear route lacks a path or contains a query error')
        for contact in candidate['rigidGraspContacts']:
            require(len(contact['contacts'])>=2 and all(-1e-8<=d<=1e-6 for d in contact['distancesMetres']), 'Invalid rigid grasp contacts')
        digest(cell/(name+'-cell.mtrg'))
    load = read(cell/'registered-load-report.json')
    require(load['configurations']>0 and load['analysisSha256']==digest(ROOT/'registered-load-study.py'), 'Changed registered load study')
    for path, expected in load['inputs'].items():
        require(digest(ROOT.parent/path)==expected, 'Stale registered load input')
    require(load['states'][-1]['ratioToCatalogRated'][1]>1, 'Unresolved shoulder finding was lost')
    for file in [BUILD/'comparison/floor-preview.png',BUILD/'comparison/floor-closeup.png',cell/'registered-cells.png',
                 ARM/'build/straight/zeroerr5-straight80-preview.png',ARM/'build/straight/zeroerr5-straight80-closeup.png']:
        digest(file)
    for directory in (ROOT, ARM):
        for file in directory.iterdir():
            if file.is_file() and file.suffix in ('.hx','.py','.cpp','.json','.sh','.md'):
                digest(file)
    digest(ROOT.parents[2]/'app/tests/reference-arms/ReferenceArmsApplicationTests.hx')
    result = dict(status='current_artifact_integrity_and_review_coverage_pass',referenceModels=len(models),sourceFkIkConfigurations=round_trips,
                  combinedSceneInstances=comparison['occurrences'],exactCadConfigurations=motion['uniqueObservedPoses'],
                  continuouslyCheckedCandidateTransfers=expected_routes,
                  physicalQualification=False,completeTendingCycle=False,shoulderSelectedForFullTarget=False,
                  sourceGaps=['Official FR3-WML/WMS URDF undeclared tool_Link','UR STEP login access','FAIRINO redistribution permission unresolved'],
                  files=checked)
    (BUILD/'completion-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))


if __name__ == '__main__':
    main()
