"""Compare complete nine-pose modal/static reports without inventing release limits."""
import argparse
import json
import math
from pathlib import Path
from packet import digest,finite
from modal_motion import analyze as analyze_motion,markdown as motion_markdown,stream_digest


def mac(a,b):
    denominator=sum(x*x for x in a)*sum(y*y for y in b)
    return sum(x*y for x,y in zip(a,b))**2/denominator if denominator else 0.0


def collect_modal(directory,config_path=Path(__file__).with_name("frame-study.json")):
    paths=sorted(p for p in directory.iterdir() if p.is_dir() and (p/'modal/report.json').exists())
    if len(paths)!=9:raise ValueError('Nine modal reports required')
    center=json.loads((directory/'center/modal/report.json').read_text());rows=[];sources={}
    if digest(config_path)!=center['configurationSha256']:raise ValueError('Configuration file differs from modal report')
    configuration=json.loads(config_path.read_text())
    for pose in paths:
        path=pose/'modal/report.json';modal=json.loads(path.read_text());finite(modal)
        if modal.get('solverThreads')!=1:raise ValueError('Modal sweep requires qualified serial solver reports')
        motion_path=pose/'modal/mass-motion.json'
        motion=json.loads(motion_path.read_text()) if motion_path.exists() else None
        valid=motion is not None and motion['sourceReportSha256']==digest(path) and motion['postprocessorSha256']==digest(Path(__file__).with_name('modal_motion.py'))
        if valid:
            size=modal['meshes'][-1]['sizeMm']
            valid=all(stream_digest(pose/f'modal/mesh-{size:g}'/name/'frame.frd')==source['frdSha256'] for name,source in motion['sources'].items())
        if not valid:
            motion=analyze_motion(path)
            motion_path.write_text(json.dumps(motion,indent=2)+'\n')
            (motion_path.parent/'mass-motion.md').write_text(motion_markdown(motion))
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
                'columnBendingResidualRatios':[m['columnBendingResidualOverMaxCastingMotion'] for m in modes],
                'lowestColumnBendingProxyHz':min((m['frequencyHz'] for m in column),default=None),
                'centerModeMatches':matches,'massMotion':motion['supports'][name]}
        rows.append({'pose':pose.name,'jointsMm':modal['jointsMm'],'manifestSha256':modal['manifestSha256'],'supports':supports})
    return {'status':'Nine-pose upper-bound modal numerical gates passed; operating-band acceptance remains open',
        'configuration':configuration,'configurationSha256':center['configurationSha256'],'poses':rows,'sourceReportSha256':sources,
        'postprocessorSha256':digest(Path(__file__)),
        'massMotionPostprocessorSha256':digest(Path(__file__).with_name('modal_motion.py')),'modeTracking':'Nearest sampled casting positions, unweighted shape MAC; inspect crossings/degenerate subspaces and low MAC before identifying a mode. No automatic acceptance threshold.'}


def modal_markdown(report):
    lines=['# Nine-pose undamped upper-bound modes','',report['status'],'',
        '| Pose | Support | First six frequencies, Hz | Lowest column-bending proxy, Hz |',
        '| --- | --- | --- | --- |']
    for row in report['poses']:
        for name,support in row['supports'].items():
            values=', '.join(f'{v:.3f}' for v in support['frequenciesHz'][:6])
            column=support['lowestColumnBendingProxyHz']
            column_text='Not in requested modes' if column is None else f'{column:.3f}'
            lines.append(f'| [{row["pose"]}]({row["pose"]}/modal/report.md) | {name} | {values} | {column_text} |')
    lines+=['','## Meshed material and support assumptions','',
        '| MaterialLibrary ID | E, GPa | Poisson ratio | Density, kg/m³ |','| --- | --- | --- | --- |']
    for name,material in report['configuration']['materials'].items():
        lines.append(f'| {name} | {material["youngsPa"]/1e9:g} | {material["poissonRatio"]:g} | {material["densityKgPerM3"]:g} |')
    for name,support in report['configuration']['supportConfigurations'].items():
        lines+=['',f'{name}: {support["description"]}; vertical spring {support["verticalSpringNPerMm"]:g} N/mm per foot. Constraints: {support["constraints"]}.']
    lines+=['','Both configurations leave foot rotations free. Support labels do not imply an ordering of every mode frequency; compare shapes and participation.',
        '',report['modeTracking'],'','## Center mode tracking','',
        '| Pose | Support | Center mode | Matched mode | MAC | Frequency change |','| --- | --- | --- | --- | --- | --- |']
    for row in report['poses']:
        if row['pose']=='center':continue
        for name,support in row['supports'].items():
            for match in support['centerModeMatches'][:8]:
                lines.append(f'| {row["pose"]} | {name} | {match["centerMode"]} | {match["poseMode"]} | {match["sampledShapeMAC"]:.4f} | {100*(match["frequencyRatio"]-1):+.2f}% |')
    lines+=['','## Coupled mass motion','',
        '| Pose | Support | Mode | Hz | Largest lumped kinetic target | Its kinetic fraction | Solid kinetic fraction |',
        '| --- | --- | --- | --- | --- | --- | --- |']
    for row in report['poses']:
        for name,support in row['supports'].items():
            for mode in support['massMotion'][:6]:
                group=mode['massGroups'][0]
                lines.append(f'| [{row["pose"]}]({row["pose"]}/modal/mass-motion.md) | {name} | {mode["mode"]} | {mode["frequencyHz"]:.3f} | {group["target"]} | {100*group["kineticFraction"]:.2f}% | {100*mode["solidKineticFraction"]:.2f}% |')
    lines+=['','## Excitation frequency equality (no operating band assumed)','',
        '| Support | Mode order | Frequency range, Hz | Rotation / one flute, rpm | Two flutes, rpm | Four flutes, rpm |',
        '| --- | --- | --- | --- | --- | --- |']
    for name in report['poses'][0]['supports']:
        for i in range(3):
            frequencies=[p['supports'][name]['frequenciesHz'][i] for p in report['poses']]
            lo,hi=min(frequencies),max(frequencies)
            ranges=[f'{60*lo/n:.0f}–{60*hi/n:.0f}' for n in (1,2,4)]
            lines.append(f'| {name} | {i+1} | {lo:.3f}–{hi:.3f} | '+ ' | '.join(ranges)+' |')
    lines+=['','These are frequency-equality speeds f = rpm × flute count / 60, not exclusion bands or predicted response. Declared spindle/motor/axis operating bands, forcing harmonics, damping and joint uncertainty remain necessary for physical acceptance. Mode order is shown for reference; use the MAC table for identity.']
    lines+=['','Kinetic localization complements the casting shape proxy; it does not locate strain energy or assign a pure column/mount mode automatically.',
        '','The column-proxy flag uses an arbitrary residual ratio of 0.1. A low support/frame mode can cross that threshold with travel; a change in the first flagged frequency does not establish a new low column mode. Compare the same shape using MAC and the continuous residual ratios in the JSON.',
        '','Mode shape SVGs and effective masses are retained in each pose/modal report. Bonded interfaces and rigid axes raise mode frequencies. No damping or forced-response amplitudes are predicted.']
    return '\n'.join(lines)+'\n'


def collect(directory,config_path=Path(__file__).with_name("frame-study.json")):
    poses=sorted(p for p in directory.iterdir() if p.is_dir() and (p/'modal/report.json').exists())
    if len(poses)!=9:raise ValueError('Nine modal reports required')
    target=None
    rows=[];center=json.loads((directory/'center/modal/report.json').read_text());sources={}
    if digest(config_path)!=center['configurationSha256']:raise ValueError('Configuration file differs from study reports')
    configuration=json.loads(config_path.read_text())
    for pose in poses:
        modal_path=pose/'modal/report.json';modal=json.loads(modal_path.read_text());finite(modal)
        if modal.get('solverThreads')!=1:raise ValueError('Modal sweep requires qualified serial solver reports')
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
                path=pose/case/'report.json';static=json.loads(path.read_text());finite(static);sources[str(path)]=digest(path)
                if static.get('solverThreads')!=1:raise ValueError('Static sweep requires qualified serial solver reports')
                if not static['converged'] or static['manifestSha256']!=modal['manifestSha256'] or static['configurationSha256']!=modal['configurationSha256']:
                    raise ValueError('Unconverged or different CAD packet in static/modal comparison')
                b=static['meshes'][-1]['supports'][name]['complianceBudget']
                fraction=b['frameShareTarget']['maximumFraction']
                if target is None:target=fraction
                if fraction!=target:raise ValueError('Frame targets differ across studies')
                c=b['componentComplianceUmPerN'];total=b['complianceUmPerN']
                shares={a:sum(b['frameEnergyDiagonalUmPerN'][a][g] for g in ('BASE_CASTING','COLUMN_CASTING'))/(total[i][i]-c['tool'][i][i]) for i,a in enumerate('xyz')}
                cases[case]={'castingMachineLoopShares':shares,'machineWorstLoadBoxUm':b['machineLoopExcludingTool']['worstLoadBoxCorner']['normUm'],'worstLoadBoxUm':b['worstLoadBoxCorner']['normUm']}
            column=[m for m in modes if m['columnBendingProxy']]
            supports[name]={'frequenciesHz':[m['frequencyHz'] for m in modes],
                'columnBendingResidualRatios':[m['columnBendingResidualOverMaxCastingMotion'] for m in modes],
                'lowestColumnBendingProxyHz':min((m['frequencyHz'] for m in column),default=None),
                'centerModeMatches':tracking,'staticCases':cases}
        rows.append({'pose':pose.name,'jointsMm':modal['jointsMm'],'supports':supports})
    maximum=max((share,r['pose'],name,case,a) for r in rows for name,s in r['supports'].items()
        for case,c in s['staticCases'].items() for a,share in c['castingMachineLoopShares'].items())
    return {'postprocessorSha256':digest(Path(__file__)),'status':'Numerical nine-pose comparison; operating-band and measured modal acceptance remain open',
        'configuration':configuration,'configurationSha256':center['configurationSha256'],'maximumCastingMachineLoopShare':{'fraction':maximum[0],'pose':maximum[1],'support':maximum[2],'case':maximum[3],'axis':maximum[4]},
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
    lines+=['','## Machine-loop static budgets','',
        '| Pose | Support | Reference | Casting X/Y/Z, excluding tool | Machine-only worst box, µm | Total worst box, µm |',
        '| --- | --- | --- | --- | --- | --- |']
    for row in report['poses']:
        for name,support in row['supports'].items():
            for case,result in support['staticCases'].items():
                shares='/'.join(f'{100*result["castingMachineLoopShares"][a]:.2f}%' for a in 'xyz')
                lines.append(f'| {row["pose"]} | {name} | [{case}]({row["pose"]}/{case}/report.md) | {shares} | {result["machineWorstLoadBoxUm"]:.3f} | {result["worstLoadBoxUm"]:.3f} |')
    lines+=['',report['modeTracking'],'',report['lighteningDecision'],'','Full input/report hashes and mode matches are in report.json.']
    return '\n'.join(lines)+'\n'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);parser.add_argument('--modal-only',action='store_true');parser.add_argument('--config',type=Path,default=Path(__file__).with_name('frame-study.json'));args=parser.parse_args()
    if args.modal_only:
        result=collect_modal(args.directory,args.config)
        (args.directory/'modal-report.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.directory/'modal-report.md').write_text(modal_markdown(result))
        raise SystemExit(0)
    modal=collect_modal(args.directory,args.config)
    (args.directory/'modal-report.json').write_text(json.dumps(modal,indent=2)+'\n')
    (args.directory/'modal-report.md').write_text(modal_markdown(modal))
    result=collect(args.directory,args.config)
    (args.directory/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.directory/'report.md').write_text(markdown(result))
