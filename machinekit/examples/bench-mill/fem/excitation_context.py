"""Compare a declared design-speed point with the qualified modal matrix.

Frequency distance is diagnostic. Operating bands, forcing and joint/material
uncertainty determine acceptance; this reducer assigns no acceptance threshold.
"""
import argparse
import json
import math
from pathlib import Path
from packet import digest, finite


def collect(path,rpm,flutes,sources,note):
    if not math.isfinite(rpm) or rpm<=0 or flutes<1:raise ValueError('Positive rpm and flute count required')
    report=json.loads(path.read_text());finite(report)
    if len(report['poses'])!=9:raise ValueError('Nine-pose modal matrix required')
    source_hashes={Path(k).resolve():v for k,v in report['sourceReportSha256'].items()}
    mode_reports={}
    for pose in report['poses']:
        source=path.parent/pose['pose']/'modal/report.json'
        if digest(source)!=source_hashes.get(source.resolve()):raise ValueError('Changed source modal report')
        original=json.loads(source.read_text())
        if not original['converged'] or original.get('solverThreads')!=1:raise ValueError('Qualified serial modal reports required')
        mode_reports[pose['pose']]=original
    rows=[]
    for label,target in (('rotation-one-per-rev',rpm/60),('tooth-pass-context',rpm*flutes/60)):
        for pose in report['poses']:
            for name,support in pose['supports'].items():
                index,frequency=min(enumerate(support['frequenciesHz']),key=lambda item:abs(item[1]-target))
                motion=next(m for m in support['massMotion'] if m['mode']==index+1)
                meshes=mode_reports[pose['pose']]['meshes']
                coarse,fine=(m['supports'][name]['modes'][index]['frequencyHz'] for m in meshes[-2:])
                rows.append({'orderedFrequenciesLastTwoMeshesHz':[coarse,fine],
                    'lastRefinementRelativeChange':abs(fine/coarse-1),'excitation' :label,'targetHz':target,'pose':pose['pose'],'support':name,
                    'mode':index+1,'frequencyHz':frequency,'distanceHz':abs(frequency-target),
                    'relativeFrequencyDistance':abs(frequency-target)/target,
                    'machineReferenceMotion':motion['machineReferenceMotion'],
                    'solidKineticFraction':motion['solidKineticFraction']})
    return {'acceptance':None,'referenceRpm':rpm,'contextFlutes':flutes,'note':note,
        'modalReport':str(path),'modalReportSha256':digest(path),'postprocessorSha256':digest(Path(__file__)),
        'sourceSha256':{str(p):digest(p) for p in sources},'nearestPerPose':rows,
        'limitations':'Closest computed frequencies among 12 modes; normalized shape motion is not forced response. Actual cutting/motor/axis bands, harmonics, damping and joint/material uncertainty remain open.'}


def markdown(report):
    lines=['# Design-speed excitation context','',report['note'],'',
        f'Reference speed: {report["referenceRpm"]:g} rpm; explicit contextual flute count: {report["contextFlutes"]}.',
        '', '| Excitation | Support | Closest pose | Mode | Target Hz | Mode Hz | Frequency distance | Last refinement change |',
        '| --- | --- | --- | --- | --- | --- | --- | --- |']
    groups={}
    for row in report['nearestPerPose']:
        key=row['excitation'],row['support']
        if key not in groups or row['distanceHz']<groups[key]['distanceHz']:groups[key]=row
    for row in groups.values():
        lines.append(f'| {row["excitation"]} | {row["support"]} | {row["pose"]} | {row["mode"]} | {row["targetHz"]:.3f} | {row["frequencyHz"]:.3f} | {100*row["relativeFrequencyDistance"]:.3f}% | {100*row["lastRefinementRelativeChange"]:.3f}% |')
    lines+=['','Frequency clearance needs a numerical and material/joint uncertainty margin; a small distance alone does not establish detuning.', '',report['limitations'],'','The JSON retains every pose and the normalized nose/bar relative-motion ratios.',
        '',f'Modal aggregate SHA256: `{report["modalReportSha256"]}`.']
    return '\n'.join(lines)+'\n'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('modal_report',type=Path)
    p.add_argument('--rpm',type=float,required=True);p.add_argument('--flutes',type=int,required=True)
    p.add_argument('--source',type=Path,action='append',required=True);p.add_argument('--note',required=True);a=p.parse_args()
    r=collect(a.modal_report,a.rpm,a.flutes,a.source,a.note)
    (a.modal_report.parent/'excitation-context.json').write_text(json.dumps(r,indent=2)+'\n')
    (a.modal_report.parent/'excitation-context.md').write_text(markdown(r))
