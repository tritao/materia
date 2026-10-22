"""Classify pinned J4 histories against J5; do not treat filenames as ratios.

Consumes immutable planner exports and the archived brace-study replay results.
No controlled J4 history is admitted as a ready-stop/executor-clock J5 history.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np

METRICS=('peakContourErrorMm','movingHandoverPeakMm','residualAfterStopMm')
def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--j4',type=Path,required=True);p.add_argument('--replays',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();sys.path.insert(0,str(a.source));from curved_error import load_export
    exports=[];baseline=[];diagnostic=[];fatigue=[]
    for bed in (2600,3150):
        for tag in ('nominal','override-1p5','override-3','hold-resume-1p5','hold-resume-3'):
            prefix=a.j4/str(bed)/('scale-1-'+tag)
            meta,rows,manifest=load_export(prefix,prefix.with_suffix('.manifest.json'))
            config=json.loads(prefix.with_suffix('.config.json').read_text())
            control=config.get('control')
            assert {k:v for k,v in config.items() if k!='control'}==meta['config'],(bed,tag,'config mismatch')
            assert control==meta.get('control'),(bed,tag,'control mismatch')
            if tag!='nominal':
                assert control['feedRatio']==.75
                assert control['atSeconds']==(1.5 if tag.endswith('1p5') else 3.)
                assert 'Offline ordinary compiler prefix/splice/tail' in meta['controlScope']
            limits=np.array([2/3,2/3,3,3,config['jerkX'],config['jerkY']])
            maxima=np.max(np.abs(rows[:,3:9]),axis=0)
            # Continuous certificates are checked by load_export; samples are
            # an independent gross-envelope audit, not their replacement.
            assert np.all(maxima<=limits*(1+1e-6)+1e-6),(bed,tag,maxima,limits)
            exports.append(dict(bedMm=bed,history=tag,control=control,
                envelopeClass='nominal' if control is None else 'excluded-offline-control',
                excludesWhy=None if control is None else 'Not approved J5 ready-stop/executor-clock export',
                durationS=meta['durationS'],sampledAbsMax=dict(zip(('vx','vy','ax','ay','jx','jy'),maxima.tolist())),
                sha256={suffix:digest(prefix.with_suffix(suffix)) for suffix in ('.csv','.json','.manifest.json','.config.json')}))
        files=list((a.replays/'motion-coarse').glob(f'y{bed}-baseline-*.json'))
        for mesh in (100,50):
            for metric in METRICS:
                for nominal in (True,False):
                    candidates=[]
                    for f in files:
                        d=json.loads(f.read_text())
                        if d['meshMm']!=mesh:continue
                        for h in d['histories']:
                            if nominal and h['history']!='nominal':continue
                            candidates.append(dict(valueMm=h['worst'][metric]['value'],poseMm=d['pose'],history=h['history'],meshMm=mesh,
                                frequencyScale=h['worst'][metric]['frequencyScale'],dampingRatio=h['worst'][metric]['dampingRatio'],
                                source=f.name,sourceSha256=digest(f)))
                    if candidates:
                        r=dict(bedMm=bed,metric=metric,**max(candidates,key=lambda r:r['valueMm']))
                        r.update(passesReserve=r['valueMm']<=.04,passesBudget=r['valueMm']<=.05)
                        (baseline if nominal else diagnostic).append(r)
    for f in sorted((a.replays/'weld-motion').glob('*baseline*.json')):
        d=json.loads(f.read_text())
        for nominal in (True,False):
            rows=[dict(rangeMPa=c['fatigueRangeMPa'],history=h['history'],frequencyScale=h['frequencyScale'],dampingRatio=h['dampingRatio'],group=c['group'],pieces=c['pieces'])
                  for h in d['fatigue'] if not nominal or h['history']=='nominal' for c in h['contacts']]
            if rows:fatigue.append(dict(source=f.name,sourceSha256=digest(f),poseMm=d['pose'],nominalOnly=nominal,**max(rows,key=lambda r:r['rangeMPa'])))
    out=dict(policy={'hold':'ready jerk-limited published stop; normal planned resume',
        'overrideAtOrBelowOne':'jerk-filtered executor clock; certify k derivatives with reserved V/A/J',
        'overrideAboveOne':'next-window raised-feed replan within unchanged V/A/J and maximum feed; no k>1 clock scaling',
        'missingCoverage':['J5 ready-stop/filtered-clock histories','raised-feed next-window replans'],
        'status':'Nominal subset only; no full-envelope release'},exports=exports,nominalBaseline=baseline,oldControlDiagnostic=diagnostic,fatigue=fatigue)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
    print('PASS: 10 immutable exports; sampled bounds and continuous certificates checked; control histories excluded from J5 acceptance')
if __name__=='__main__':main()
