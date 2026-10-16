#!/usr/bin/env python3
"""Collect current original-arm refinement and supplier evidence, preserving baseline."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ARM = ROOT.parent/'zeroerr-arm'
CELL = ROOT/'build/cell-registration'
inputs = {}


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


def main():
    baseline_path = CELL/'original-registration-planned.json'
    baseline = read(baseline_path)
    refined_path = CELL/'original-registration-refinement-planned.json'
    refined = read(refined_path)
    require(refined['baselineSha256'] == digest(baseline_path), 'Changed baseline')
    for key, name in [('refinementProducerSha256','OriginalRouteRefinement.hx'),
                      ('portalProducerSha256','OriginalPortalWaypoints.hx'),
                      ('outfeedProducerSha256','OriginalOutfeedBranches.hx'),
                      ('registrationProducerSha256','CellRegistration.hx')]:
        require(refined[key] == digest(ROOT/name), 'Stale native producer: '+name)
    require(refined['fixtureSha256'] == baseline['fixtureSha256'] == digest(CELL/'fixtures.json'), 'Changed fixtures')
    require(refined['configSha256'] == baseline['configSha256'] == digest(ROOT/'tending-registration-config.json'), 'Changed study policy')
    require(len(refined['models']) == 1, 'Expected one original arm')
    model = refined['models'][0]
    require(model['model'] == 'zeroerr-original' and model['sourceFingerprint'] == digest(ARM/'build/zeroerr5.mtrg'), 'Changed original geometry')
    candidate = model['candidates'][0]
    old = baseline['models'][0]['candidates'][0]
    require(candidate['baseWorldMetres'] == old['baseWorldMetres'], 'Changed base')
    permitted = {}
    branch_selections = []
    for row in candidate['outfeedBranchReviews']:
        selection = row['selection']
        if selection['clearance'] is None:
            continue
        for suffix, field, original_field in [('clearance','clearance','originalClearance'),('approach','approach','originalApproach')]:
            key = (row['station']+'-'+suffix,row['channel'])
            permitted[key] = (row[original_field],selection[field])
        for field in ['clearancePositionError','clearanceAngleError','approachPositionError','approachAngleError']:
            require(0 <= selection[field] <= 1e-6, 'Changed Cartesian target')
        for leg in ['transfer','descent']:
            require(selection[leg]['clear'] and selection[leg]['error'] is None, 'Branch pair lacks continuous proof')
        branch_selections.append(dict(station=row['station'],channel=row['channel'],clearanceCandidates=selection['clearanceCandidates'],
                                      approachCandidates=selection['approachCandidates'],checks=selection['checks']))
    require(len(candidate['poses']) == len(old['poses']) == 82, 'Changed target inventory')
    for before, after in zip(old['poses'],candidate['poses']):
        require({k:v for k,v in before.items() if k!='q'} == {k:v for k,v in after.items() if k!='q'}, 'Changed target semantics')
        if before['q'] != after['q']:
            require(permitted.get((before['name'],before['channel'])) == (before['q'],after['q']), 'Unreviewed target branch change')
    require(len(candidate['routes']) == len(old['routes']) == 23, 'Incomplete transfer coverage')
    poses = {(p['name'],p['channel']):p['q'] for p in candidate['poses']}
    recovered, unresolved, retained = [], [], []
    for index, (before, after) in enumerate(zip(old['routes'],candidate['routes'])):
        for key in ['station','tool','fromTool','fromPose','toPose','door','states']:
            require(before[key] == after[key], 'Changed request/diagnostic states: '+str(index))
        was_clear = (before.get('continuous') or {}).get('clear') is True
        proof = after.get('continuous') or {}
        clear = proof.get('clear') is True
        row = dict(index=index,station=after['station'],fromTool=after['fromTool'],tool=after['tool'],
                   fromPose=after['fromPose'],toPose=after['toPose'],door=after['door'],planner=after['planner'])
        if clear:
            path = after['path']
            require(path and len(path)>=2 and proof['error'] is None and proof['queries']>0 and proof['failedWorld']==-1, 'Invalid continuous proof')
            start = poses[('outside staging' if after['fromPose']=='ready' else after['fromPose'],after['fromTool'])]
            end = poses[(after['toPose'],after['tool'])]
            require(path[0] == start and path[-1] == end, 'Changed endpoint joint branch')
            require(all(len(q)==6 and all(math.isfinite(v) and -math.pi<=v<=math.pi for v in q) for q in path), 'Invalid study joint values')
            row.update(waypoints=len(path),queries=proof['queries'])
            (retained if was_clear else recovered).append(row)
        else:
            require(not was_clear, 'Previously checked transfer regressed')
            row['baselineWitness'] = before.get('continuous')
            unresolved.append(row)
    portal = candidate['portalWaypoints']
    require(len(portal['targets']) == 18 and len(portal['points']) == sum(t['clearInFourOpenDoorStates'] for t in portal['targets']), 'Incomplete entry-waypoint IK evidence')
    load = read(CELL/'original-refined-registered-load-report.json')
    require(load['baseWorldMetres'] == candidate['baseWorldMetres'], 'Different load-study base')
    require(load['analysisSha256'] == digest(ROOT/'registered-load-study.py'), 'Stale load analysis')
    for path, expected in load['inputs'].items():
        require(digest(ROOT.parent/path) == expected, 'Changed refined load input')
    supplier = read(ARM/'build/original-evaluation/supplier/review.json')
    require(supplier['producerSha256'] == digest(ARM/'original-supplier-review.py'), 'Stale supplier reviewer')
    require(supplier['moduleFamiliesConfirmedByAssemblyDrawing'] and supplier['unresolved'], 'Missing supplier boundaries')
    for path, expected in supplier['inputs'].items():
        require(digest(Path(path)) == expected, 'Changed supplier source snapshot')
    request = read(ARM/'original-supplier-request.json')
    require(request['status'] == 'draft_not_sent_not_ordered' and request['originalCellLoadReportSha256'] == digest(CELL/'original-registered-load-report.json'), 'Stale or released supplier request')
    require(request['refinedCellLoadReportSha256'] == digest(CELL/'original-refined-registered-load-report.json'), 'Stale refined supplier sizing request')
    require(all(j['completeOrderingCode'] is None for j in request['jointFamilies']), 'Unconfirmed ordering code selected')
    result = dict(status='original_route_refinement_reviewed_not_hardware_qualified',baselineTransfers=len(retained),
                  refinedTransfers=len(retained)+len(recovered),recovered=recovered,unresolved=unresolved,
                  retainedRoutesFreshlyChecked=len(retained),entryWaypointConfigurations=len(portal['points']),
                  outfeedBranchSelections=branch_selections,
                  outfeedBranchAttempts=[dict(station=r['station'],channel=r['channel'],
                    clearanceCandidates=r['selection']['clearanceCandidates'],approachCandidates=r['selection']['approachCandidates'],
                    checks=r['selection']['checks']) for r in candidate['outfeedBranchReviews']],
                  evaluatedLoadConfigurations=load['configurations'],shoulderScreen=load['states'][-1],
                  supplierStatus=supplier['status'],physicalQualification=False,completeTendingCycle=False,
                  scope='Same original geometry, base, Cartesian targets, inventory, margins and four holding states. Reviewed outfeed IK branch changes retain target position/orientation within 1e-6 m/rad. Fresh continuous proofs on retained and recovered paths; bounded reviewed-pose roadmap. Paths are stop-to-stop quintics, not a timed/contact tending cycle.',
                  producerSha256=digest(Path(__file__)),inputs=inputs)
    output = ARM/'build/original-evaluation/refinement-review.json'
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['inputs','shoulderScreen']},indent=2))


if __name__ == '__main__':
    main()
