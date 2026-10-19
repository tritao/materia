#!/usr/bin/env python3
"""Check native audit provenance and unexpected interference without treating contacts as clearances."""
import argparse
import hashlib
import json
import math
from pathlib import Path

parser=argparse.ArgumentParser();selection=parser.add_mutually_exclusive_group();selection.add_argument('--current-access',action='store_true');selection.add_argument('--vise-access',action='store_true');args=parser.parse_args()
current_access=args.current_access or args.vise_access
report_dir='electric-expert-vise' if args.vise_access else 'electric-expert-access' if current_access else 'electric-access'
pose_file='vise-tending-pose-audit.json' if args.vise_access else 'current-tending-pose-audit.json' if current_access else 'tending-pose-audit.json'
root=Path(__file__).resolve().parent/'build/expert' 
layout=json.loads((root/'layout-report.json').read_text())
tool=json.loads((root/'tool-cad-audit.json').read_text())
poses=json.loads((root/pose_file).read_text())
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
assert digest(root/'dual-expert-tool.mtrg')==poses['sourceSceneSha256'], 'Scene changed after audit'
assert digest(root/'dual-expert-tool.mtrg')==tool['sourceSceneSha256'], 'Retained tool audit has a different scene'
for source in poses['sources']:
    assert digest(root/source['id'])==source['sha256'], f"Changed service STEP: {source['id']}"
for source in poses['armSources']:
    assert digest(root/poses.get('armSourceDirectory','robot-source')/(source['id']+'.step'))==source['sha256'], f"Changed robot STEP: {source['id']}"
assert digest(root.parents[2]/('machine-tending/build/'+report_dir+'/report.json'))==poses['sourceReportSha256'], 'Saved pose source changed'
assert layout['sourceSceneSha256']==poses['sourceSceneSha256'], 'Service-pair report has a different scene'
assert layout['unexpectedInterferences']==0, 'Unexpected service/service overlap'
assert len(layout['fastenerTrials'])==2 and all(x['overlapMm3']<=.001 for x in layout['fastenerTrials']), 'Fastener trial failed'
intended={('expert-saddle','adapter'),('a-electrical-cable','a-pgc-140-50'),('b-electrical-cable','b-pgc-140-50')}
planned_connections=[('a-electrical-cable','a-service-coupler'),('b-electrical-cable','b-service-coupler'),
                     ('a-service-coupler','expert-branch-a'),('b-service-coupler','expert-branch-b'),
                     ('expert-splitter','expert-branch-a'),('expert-splitter','expert-branch-b'),
                     ('expert-splitter','expert-power-lead'),('expert-splitter','expert-signal-lead'),
                     ('expert-signal-lead','expert-wrist-elbow'),('expert-saddle','expert-saddle-pad'),('expert-saddle-pad','expert-splitter'),
                     ('expert-strap-56','expert-splitter'),('expert-strap-76','expert-splitter'),
                     ('expert-strap-56','expert-saddle'),('expert-strap-76','expert-saddle')]
missing_connections=[]
for a,b in planned_connections:
    pair=next((x for x in layout['checks'] if {x['a'],x['b']}=={a,b}),None)
    if pair is None or pair['distanceMm']>.01:missing_connections.append([a,b])
for pair in intended:
    matches=[x for x in tool['checks'] if (x['service'],x['tool'])==pair]
    if not matches or any(x['distanceMm']>.01 for x in matches):missing_connections.append(list(pair))
tool_bad=[x for x in tool['checks'] if x['overlapMm3']>.1 or (x['distanceMm']<.5 and (x['service'],x['tool']) not in intended)]
robot_bad=[x for x in poses['checks'] if x['overlapMm3']>.1 or x['distanceMm']<5]
nearest=min(poses['checks'],key=lambda x:x['distanceMm'])
source=json.loads((root.parents[2]/('machine-tending/build/'+report_dir+'/report.json')).read_text())
if current_access:
    assert source['provenance']['toolSha256']==poses['sourceSceneSha256'], 'Access approvals used different tool geometry'
cases={}
for row in source['results']:
    if row['profile']!='tending-top-entry':continue
    key=row['station']+'/'+row['side'];cases.setdefault(key,False)
    if row['geometricClear'] and row['ikStatus']=='sampled route solved and clear':cases[key]=True
blocked=[key for key,clear in cases.items() if not clear]
if args.vise_access:
    assert set(cases)=={'vise-seat/a','vise-seat/b','vise-load/a','vise-load/b'} and all(cases.values()), 'Vise study is incomplete'
    assert source['exactViseNarrowPhase'], 'Vise approvals need native CAD refinement'
    for row in source['results']:
        if row['ikStatus']!='sampled route solved and clear':continue
        trace=row['routeQ'];path=row['path']
        assert len(trace)==len(path)>1 and trace[-1]==row['q'], 'Saved final pose differs from the actual route endpoint'
        assert all(len(q)==6 and all(math.isfinite(v) for v in q) for q in trace), 'Invalid joint trace'
        assert all(max(abs(a-b) for a,b in zip(q0,q1))<=.35 for q0,q1 in zip(trace,trace[1:])), 'Joint branch jump in accepted trace'
        assert all(math.dist([a[k] for k in ['x','y','z']],[b[k] for k in ['x','y','z']])<=source['maximumSampleStepMm']+.000001 for a,b in zip(path,path[1:])), 'Cartesian sample gap exceeds study bound'
    exact_root=root.parents[2]/'machine-tending/build'/report_dir
    exact=json.loads((exact_root/'exact-vise-checks.json').read_text())
    assert exact['toolSha256']==source['provenance']['toolSha256'], 'Exact query trace uses another tool'
    for item in exact['sources']:
        assert digest(exact_root/item['id'])==item['sha256'], f"Changed exact CAD source: {item['id']}"

summary={'status':'Static CAD review passed' if not tool_bad and not robot_bad and not missing_connections else 'Interference or inadequate clearance found',
         'fastenerTrials':len(layout['fastenerTrials']),'newServicePairs':len(layout['checks']),'retainedToolPairs':len(tool['checks']),
         'sourceApprovedOnly':poses.get('approvedOnly',False),'sourceApprovedCases':sum(cases.values()),
         'sourceRequiredCases':len(cases),'sourceUnresolvedCases':blocked,
         'savedRobotPoses':poses['uniquePoses'],'robotPairs':len(poses['checks']),
         'retainedToolMarginMm':.5,'robotMarginMm':5,
         'missingPlannedContacts':missing_connections,'unexpectedToolPairs':tool_bad,'unexpectedRobotPairs':robot_bad,'nearestRobotPair':nearest,
         'limitations':['Estimated Expert dimensions and wiring; OEM mating not qualified',
                        'Static saved poses only; no swept motion, live cell access or moving dress-pack certificate',
                        'Bounds distances are lower limits; exact CAD used for pairs within 5 mm',
                        'Housing supplier surfaces do not establish closed-volume containment',
                        'Only the documented cable exits and saddle/adapter mounting faces are intended retained-tool contacts']}
(root/('vise-review-summary.json' if args.vise_access else 'current-review-summary.json' if current_access else 'review-summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ['unexpectedToolPairs','unexpectedRobotPairs','limitations']},indent=2))
assert not tool_bad and not robot_bad and not missing_connections, 'See review-summary.json for unexpected pairs'
