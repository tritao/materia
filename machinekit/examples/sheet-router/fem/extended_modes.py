"""Resolve forty modes and report up to six gantry-dominant modes per governing pose."""
import argparse
import hashlib
import json
from pathlib import Path
from sensitivity import first_gantry
from study import solve


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--work-y',type=int,choices=(2600,3150));parser.add_argument('--packets',required=True,type=Path);parser.add_argument('--ccx',required=True,type=Path);parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    data=json.loads((args.packets/'results.json').read_text());args.output.mkdir(parents=True,exist_ok=True);rows=[]
    existing=args.output/'extended-modes.json'
    previous=json.loads(existing.read_text()) if args.work_y and existing.exists() else {}
    rows=[r for r in previous.get('rows',[]) if r['workY']!=args.work_y]
    for length in ((args.work_y,) if args.work_y else (2600,3150)):
        fine=[r for r in data['results'] if r['meshMm']==50 and r['workY']==length]
        worst=min(fine,key=lambda r:first_gantry(r)['frequencyHz']);packet=json.loads((args.packets/f'y{length}'/'manifest.json').read_text());packet['floorSupport']='anchored';packet['modeCount']=40
        pair=[]
        for h in (100,50):
            result=solve(packet,worst['pose'],h,args.ccx.resolve(),args.output/f'y{length}-h{h}')
            dominant=[m for m in result['modalParticipation'] if m['gantryFraction']>=.5]
            if len(dominant)<3:raise ValueError('Forty modes did not resolve three gantry-dominant modes')
            pair.append(dict(workY=length,firstSixGantry=dominant[:6],gantryModesResolved=len(dominant),**result));print(length,h,[(m['mode'],m['frequencyHz']) for m in dominant[:6]],flush=True)
        error=max(abs(a/b-1) for a,b in zip(pair[0]['firstModesHz'],pair[1]['firstModesHz']))
        if error>.05:raise ValueError('Forty-mode convergence exceeds 5%')
        # Classification near 50% can swap membership; retain all raw modes and verify frequencies by index.
        rows.append(dict(workY=length,allFortyModesRelativeChange=error,results=pair))
        sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('study.py'),Path(__file__).with_name('modes.py'),Path(__file__).with_name('front_guides.py')]}
        (args.output/'extended-modes.json').write_text(json.dumps(dict(sourceSha256=sources,previousExtractionSourceSha256=previous.get('sourceSha256'),rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
