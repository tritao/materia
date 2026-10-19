"""Recover mass-group motion/kinetic fractions from completed CCX modal FRD files.

This keeps the original solve reports immutable. It distinguishes a large
lumped-assembly motion from the casting-only shape proxy; it does not assign
physical mode names automatically.
"""
import hashlib
import argparse
import json
import math
from pathlib import Path
import re
from packet import digest,finite


def stream_digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def mass_nodes(path):
    rows=path.read_text().splitlines();result={}
    for i,line in enumerate(rows):
        match=re.fullmatch(r'\*ELEMENT,TYPE=MASS,ELSET=LUMP(\d+)_(\d+)',line)
        if match:result[int(match[1]),int(match[2])]=int(rows[i+1].split(',')[1])
    if not result:raise ValueError('No modal point-mass elements in deck')
    return result


def frd_displacements(path,wanted):
    metadata={};active=None;result=[]
    with path.open() as stream:
        for line in stream:
            raw=line.split()
            if raw and raw[0]=='1PSTEP':metadata={}
            elif raw and raw[0]=='1PMODE':metadata['mode']=int(raw[1])
            elif raw and raw[0]=='1PGM':metadata['generalizedMass']=float(raw[1])
            elif raw and raw[0]=='100CL':metadata['frequencyHz']=float(raw[2])
            elif line.startswith(' -4') and 'DISP' in line:active={**metadata,'u':{}}
            elif active is not None and line.startswith(' -1'):
                node=int(line[3:13])
                if node in wanted:active['u'][node]=[float(line[13+12*i:25+12*i]) for i in range(3)]
            elif active is not None and line.startswith(' -3'):
                result.append(active);active=None
    return result


def analyze(report_path):
    report=json.loads(report_path.read_text());finite(report)
    if not report['converged']:raise ValueError('Converged modal report required')
    mesh=report['meshes'][-1];results={};sources={}
    for name,support in mesh['supports'].items():
        directory=report_path.parent/f'mesh-{mesh["sizeMm"]:g}'/name
        deck=directory/'frame.inp';frd=directory/'frame.frd'
        if digest(deck)!=support['model']['inputSha256'] or digest(directory/'frame.dat')!=support['resultSha256']:
            raise ValueError('Modal result/deck differs from report')
        ids=mass_nodes(deck);model=support['model'];attachments=model['massAttachments'];rows=[]
        # Use a real casting node to cross-check FRD scaling against .dat output.
        check_nodes=[int(max(m['sensorDisplacements'],key=lambda n:sum(v*v for v in m['sensorDisplacements'][n]))) for m in support['modes']]
        fields=frd_displacements(frd,set(ids.values())|set(check_nodes))
        if len(fields)!=len(support['modes']):raise ValueError('Incomplete modal FRD displacement fields')
        for mode,field,check_node in zip(support['modes'],fields,check_nodes):
            finite(field)
            if field['mode']!=mode['mode'] or abs(field['frequencyHz']/mode['frequencyHz']-1)>2e-6:
                raise ValueError('FRD mode/frequency mismatch')
            normalization=field['generalizedMass']
            if normalization<=0:raise ValueError('Invalid generalized mass normalization')
            peak=max(math.sqrt(sum(v*v for v in u)) for u in mode['sensorDisplacements'].values())
            if math.dist(field['u'][check_node],mode['sensorDisplacements'][str(check_node)])/peak>2e-5:
                raise ValueError('FRD and .dat eigenvector scaling disagree')
            groups=[]
            for index,attachment in enumerate(attachments):
                weights=attachment['pointMasses'];vectors=[field['u'][ids[index,i]] for i in range(len(weights))]
                mass=sum(p['massKg'] for p in weights)
                motion=sum(p['massKg']*sum(v*v for v in u) for p,u in zip(weights,vectors))
                centre=[sum(p['massKg']*u[i] for p,u in zip(weights,vectors))/mass for i in range(3)]
                groups.append({'target':attachment['target'],'massKg':mass,'kineticFraction':motion/1000/normalization,
                    'comMotionOverMaxCastingMotion':math.sqrt(sum(v*v for v in centre))/peak,
                    'rmsMotionOverMaxCastingMotion':math.sqrt(motion/mass)/peak})
            lumped=sum(g['kineticFraction'] for g in groups)
            if lumped>1.0001:raise ValueError(f'Lumped kinetic energy exceeds generalized modal energy: {lumped}')
            rows.append({'mode':mode['mode'],'frequencyHz':mode['frequencyHz'],'generalizedMass':normalization,
                'solidKineticFraction':max(0,1-lumped),'lumpedKineticFraction':lumped,
                'massGroups':sorted(groups,key=lambda g:g['kineticFraction'],reverse=True)})
        results[name]=rows;sources[name]={'inputSha256':digest(deck),'frdSha256':stream_digest(frd)}
    return {'status':'Mass-motion diagnostic; no automatic physical mode classification',
        'sourceReportSha256':digest(report_path),'postprocessorSha256':digest(Path(__file__)),
        'normalization':'CCX FRD generalized mass; point masses converted kg to tonne; FRD displacements cross-checked against .dat',
        'supports':results,'sources':sources}


def markdown(report):
    lines=['# Modal mass motion','',report['status'],'',report['normalization'],'',
        '| Support | Mode | Hz | Largest lumped target | Its kinetic fraction | Its RMS / casting peak | Solid kinetic fraction |',
        '| --- | --- | --- | --- | --- | --- | --- |']
    for name,modes in report['supports'].items():
        for mode in modes:
            group=mode['massGroups'][0]
            lines.append(f'| {name} | {mode["mode"]} | {mode["frequencyHz"]:.3f} | {group["target"]} | {100*group["kineticFraction"]:.2f}% | {group["rmsMotionOverMaxCastingMotion"]:.3f} | {100*mode["solidKineticFraction"]:.2f}% |')
    lines+=['','Kinetic localization helps interpret the casting-only shape proxy; it does not locate strain energy. Attachment assumptions and all group fractions remain visible in mass-motion.json.']
    return '\n'.join(lines)+'\n'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('report',type=Path);args=parser.parse_args()
    result=analyze(args.report)
    (args.report.parent/'mass-motion.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.report.parent/'mass-motion.md').write_text(markdown(result))
