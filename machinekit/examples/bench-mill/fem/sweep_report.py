"""Compare complete nine-pose modal/static reports without inventing release limits."""
import argparse
import json
import math
from pathlib import Path
from packet import digest


def mac(a,b):
    return sum(x*y for x,y in zip(a,b))**2/(sum(x*x for x in a)*sum(y*y for y in b))


def collect_modal(directory):
    paths=sorted(p for p in directory.iterdir() if p.is_dir() and (p/'modal/report.json').exists())
    if len(paths)!=9:raise ValueError('Nine modal reports required')
    center=json.loads((directory/'center/modal/report.json').read_text());rows=[];sources={}
    for pose in paths:
        path=pose/'modal/report.json';modal=json.loads(path.read_text())
        if not modal['converged'] or modal['configurationSha256']!=center['configurationSha256']:
            raise ValueError('Unconverged or mixed modal configurations: '+pose.name)
        sources[str(path)]=digest(path);supports={}
        for name,result in modal['meshes'][-1]['supports'].items():
            modes=result['modes'];central=center['meshes'][-1]['supports'][name]['modes'];matches=[]
            for base in central:
                best=max(modes,key=lambda m:mac(base['shapeSignature'],m['shapeSignature']))
                matches.append({'centerMode':base['mode'],'poseMode':best['mode'],
                    'sampledShapeMAC':mac(base['shapeSignature'],best['shapeSignature']),
                    'frequencyRatio':best['frequencyHz']/base['frequencyHz']})
            column=[m for m in modes if m['columnBendingProxy']]
            supports[name]={'frequenciesHz':[m['frequencyHz'] for m in modes],
                'lowestColumnBendingProxyHz':min((m['frequencyHz'] for m in column),default=None),
                'centerModeMatches':matches}
        rows.append({'pose':pose.name,'jointsMm':modal['jointsMm'],'manifestSha256':modal['manifestSha256'],'supports':supports})
    return {'status':'Nine-pose upper-bound modal numerical gates passed; operating-band acceptance remains open',
        'configurationSha256':center['configurationSha256'],'poses':rows,'sourceReportSha256':sources,
        'postprocessorSha256':digest(Path(__file__)),
        'modeTracking':'Nearest sampled casting positions, unweighted shape MAC; inspect crossings/degenerate subspaces and low MAC before identifying a mode. No automatic acceptance threshold.'}


def modal_markdown(report):
    lines=['# Nine-pose undamped upper-bound modes','',report['status'],'',
        '| Pose | Support | First six frequencies, Hz | Lowest column-bending proxy, Hz |',
        '| --- | --- | --- | --- |']
    for row in report['poses']:
        for name,support in row['supports'].items():
            values=', '.join(f'{v:.3f}' for v in support['frequenciesHz'][:6])
            lines.append(f'| {row["pose"]} | {name} | {values} | {support["lowestColumnBendingProxyHz"]:.3f} |')
    lines+=['',report['modeTracking'],'','## Center mode tracking','',
        '| Pose | Support | Center mode | Matched mode | MAC | Frequency change |','| --- | --- | --- | --- | --- | --- |']
    for row in report['poses']:
        if row['pose']=='center':continue
        for name,support in row['supports'].items():
            for match in support['centerModeMatches'][:8]:
                lines.append(f'| {row["pose"]} | {name} | {match["centerMode"]} | {match["poseMode"]} | {match["sampledShapeMAC"]:.4f} | {100*(match["frequencyRatio"]-1):+.2f}% |')
    lines+=['','Mode shape SVGs and effective masses are retained in each pose/modal report. Bonded interfaces and rigid axes raise mode frequencies. No damping or forced-response amplitudes are predicted.']
    return '\n'.join(lines)+'\n'


def collect(directory):
    poses=sorted(p for p in directory.iterdir() if p.is_dir() and (p/'modal/report.json').exists())
    if len(poses)!=9:raise ValueError('Nine modal reports required')
    target=None
    rows=[];center=json.loads((directory/'center/modal/report.json').read_text());sources={}
    for pose in poses:
        modal_path=pose/'modal/report.json';modal=json.loads(modal_path.read_text())
        if modal['configurationSha256']!=center['configurationSha256']:raise ValueError('Modal configurations differ across poses')
        if not modal['converged']:raise ValueError('Unconverged modes: '+pose.name)
        sources[str(modal_path)]=digest(modal_path);supports={}
        for name,result in modal['meshes'][-1]['supports'].items():
            modes=result['modes'];central=center['meshes'][-1]['supports'][name]['modes']
            tracking=[]
            for base in central:
                best=max(modes,key=lambda m:mac(base['shapeSignature'],m['shapeSignature']))
                tracking.append({'centerMode':base['mode'],'poseMode':best['mode'],
                    'sampledShapeMAC':mac(base['shapeSignature'],best['shapeSignature']),
                    'frequencyRatio':best['frequencyHz']/base['frequencyHz']})
            cases={}
            for case in ('spindle-nose','test-bar-20'):
                path=pose/case/'report.json';static=json.loads(path.read_text());sources[str(path)]=digest(path)
                if not static['converged'] or static['manifestSha256']!=modal['manifestSha256'] or static['configurationSha256']!=modal['configurationSha256']:
                    raise ValueError('Unconverged or different CAD packet in static/modal comparison')
                b=static['meshes'][-1]['supports'][name]['complianceBudget']
                fraction=b['frameShareTarget']['maximumFraction']
                if target is None:target=fraction
                if fraction!=target:raise ValueError('Frame targets differ across studies')
                c=b['componentComplianceUmPerN'];total=b['complianceUmPerN']
                shares={a:sum(b['frameEnergyDiagonalUmPerN'][a][g] for g in ('BASE_CASTING','COLUMN_CASTING'))/(total[i][i]-c['tool'][i][i]) for i,a in enumerate('xyz')}
                cases[case]={'castingMachineLoopShares':shares,'worstLoadBoxUm':b['worstLoadBoxCorner']['normUm']}
            column=[m for m in modes if m['columnBendingProxy']]
            supports[name]={'frequenciesHz':[m['frequencyHz'] for m in modes],
                'lowestColumnBendingProxyHz':min((m['frequencyHz'] for m in column),default=None),
                'centerModeMatches':tracking,'staticCases':cases}
        rows.append({'pose':pose.name,'jointsMm':modal['jointsMm'],'supports':supports})
    maximum=max((share,r['pose'],name,case,a) for r in rows for name,s in r['supports'].items()
        for case,c in s['staticCases'].items() for a,share in c['castingMachineLoopShares'].items())
    return {'postprocessorSha256':digest(Path(__file__)),'status':'Numerical nine-pose comparison; operating-band and measured modal acceptance remain open',
        'maximumCastingMachineLoopShare':{'fraction':maximum[0],'pose':maximum[1],'support':maximum[2],'case':maximum[3],'axis':maximum[4]},
        'provisionalTargetFraction':target,'allCastingSharesBelowProvisionalTarget':maximum[0]<target,
        'modeTracking':'Nearest sampled casting positions, unweighted shape MAC; inspect crossings/degenerate subspaces and low MAC before identifying a mode. No automatic acceptance threshold.',
        'lighteningDecision':'Parameter proposal only after review of pose extrema and excitation bands; no CAD geometry edits',
        'poses':rows,'sourceReportSha256':sources}


def markdown(report):
    lines=['# Nine-pose upper-bound frame comparison','',report['status'],'',
        '| Pose | Support | First mode | Lowest column-bending proxy | Max casting share, no job tool |',
        '| --- | --- | --- | --- | --- |']
    for row in report['poses']:
        for name,s in row['supports'].items():
            share=max(v for c in s['staticCases'].values() for v in c['castingMachineLoopShares'].values())
            lines.append(f'| {row["pose"]} | {name} | {s["frequenciesHz"][0]:.3f} Hz | {s["lowestColumnBendingProxyHz"]} Hz | {100*share:.2f}% |')
    lines+=['',report['modeTracking'],'',report['lighteningDecision'],'','Full input/report hashes and mode matches are in report.json.']
    return '\n'.join(lines)+'\n'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);parser.add_argument('--modal-only',action='store_true');args=parser.parse_args()
    if args.modal_only:
        result=collect_modal(args.directory)
        (args.directory/'modal-report.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.directory/'modal-report.md').write_text(modal_markdown(result))
        raise SystemExit(0)
    result=collect(args.directory)
    (args.directory/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.directory/'report.md').write_text(markdown(result))
