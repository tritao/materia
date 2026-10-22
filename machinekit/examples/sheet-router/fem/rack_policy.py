"""Nominal-only timing/shaping screen on rack-seat joint modes.

Replays continuously certified ordinary-planner exports; never synthesises
above-100% executor scaling. J5 ready-stop/filtered-clock coverage stays pending.
"""
import argparse,hashlib,itertools,json,sys,gzip,shutil
from pathlib import Path
import numpy as np
import brace_motion

def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--models',type=Path,required=True);p.add_argument('--policies',type=Path,required=True);p.add_argument('--old-j4',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--beds',nargs='+',type=int,default=[2600,3150]);p.add_argument('--allow-partial',action='store_true',help='Explicit governing-pose diagnostic; never a full-bed pass');p.add_argument('--nominal-only',action='store_true');a=p.parse_args()
    sys.path.insert(0,str(a.source));from curved_error import load_export
    from contour_response import response
    result=[]
    cached={(r["bedMm"],r["policy"]):r for r in json.loads(a.output.read_text())["policies"]} if a.output.exists() else {}
    for bed in a.beds:
        base=a.old_j4/str(bed)/'scale-1-nominal';paths=[base]+[q.with_suffix('').with_suffix('') for q in sorted(a.policies.glob(f'y{bed}-*.config.json'))]
        if a.nominal_only:paths=[base]
        base_duration=None;per_pose=[]
        models=[(f,json.loads(f.read_text())) for f in sorted((a.models/f'y{bed}'/'baseline').glob('*/summary.json'))]
        if not models or len(models)!=9 and not a.allow_partial:raise ValueError('Nine structural poses required per bed unless --allow-partial')
        model_hashes={sha(f) for f,_ in models}
        for prefix in paths:
            csv=prefix.with_suffix('.csv');temporary=False
            if not csv.exists():
                compressed=csv.with_suffix('.csv.gz')
                if not compressed.is_file() or csv.parent.resolve()!=csv.parent.absolute():raise ValueError('Missing regular CSV export: '+str(csv))
                if shutil.disk_usage(csv.parent).free<8_100_000_000:raise RuntimeError('STOP: no CSV materialisation below root reserve')
                with gzip.open(compressed,'rb') as source,csv.open('xb') as destination:shutil.copyfileobj(source,destination)
                temporary=True
            try:meta,rows,manifest=load_export(prefix,prefix.with_suffix('.manifest.json'))
            finally:
                if temporary:csv.unlink()
            history_digest=manifest['outputSha256'][csv.name]
            if meta.get('control') is not None:raise ValueError('Controlled histories are not admitted to nominal policy screen')
            if base_duration is None:base_duration=meta['durationS']
            old=cached.get((bed,prefix.name))
            if old and old['trajectorySha256']==history_digest and old['manifestSha256']==sha(prefix.with_suffix('.manifest.json')) and {r['structuralSha256'] for r in old['poses']}==model_hashes:
                result.append(old);print('cached',bed,prefix.name,flush=True);continue
            dt=meta['config']['dt'];start=meta['processStartTimeS']
            steady=(np.abs(rows[:,3]-20/60)<=1e-6)&(np.max(np.abs(rows[:,4:7]),axis=1)<=1e-6)
            ends=np.flatnonzero((rows[:,0]>=start+dt)&~steady);window=float(rows[ends[0]-1,0]-start)
            observed=[]
            for f,detail in models:
                worst={k:dict(valueMm=-1.) for k in ('peakContourErrorMm','movingHandoverPeakMm','residualAfterStopMm')}
                for scale,damping in itertools.product((.85,1.,1.15),(.005,.0075,.01,.015,.02,.025,.03)):
                    r=response(detail['modes'],detail['staticAccelerationResponseMmPerMPerS2'],rows,meta['geometry'],scale,damping,dt,process_start=start,handover_window=window)
                    for k in worst:
                        if r[k]>worst[k]['valueMm']:worst[k]=dict(valueMm=r[k],frequencyScale=scale,dampingRatio=damping)
                observed.append(dict(poseMm=detail['pose'],meshMm=detail['meshMm'],structuralSha256=sha(f),worst=worst))
            maxima={k:max((dict(**r['worst'][k],poseMm=r['poseMm']) for r in observed),key=lambda q:q['valueMm']) for k in observed[0]['worst']}
            report=dict(bedMm=bed,policy=prefix.name,config=meta['config'],durationS=meta['durationS'],addedCycleTimeS=meta['durationS']-base_duration,
                worst=maxima,poses=observed,trajectorySha256=history_digest,manifestSha256=sha(prefix.with_suffix('.manifest.json')),
                scope=('Governing-pose diagnostic only; ' if a.allow_partial else 'Full nine-pose nominal screen; ')+'3 frequency samples x 7 damping values; J5 controls and continuous uncertainty acceptance pending')
            report['passesReserve']=all(v['valueMm']<=.04 for v in maxima.values());report['passesBudget']=all(v['valueMm']<=.05 for v in maxima.values())
            result.append(report);print(bed,prefix.name,meta['durationS'],{k:v['valueMm'] for k,v in maxima.items()},flush=True)
            a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(dict(policies=result),indent=2)+'\n')
    a.output.write_text(json.dumps(dict(policies=result),indent=2)+'\n')
if __name__=='__main__':main()
