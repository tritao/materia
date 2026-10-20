"""Detail nominal 5 mm stitches, retain strength assumptions, and keep shop release fail-closed."""
import argparse,csv,hashlib,json,math
from collections import defaultdict
from pathlib import Path
import numpy as np


def weld_stitches(seams,on):
    rows=[];seen=set()
    for s in seams:
        if s['type']=='Butt':continue  # No imaginary fillet at a flush wall; other accessible edges carry this joint.
        a=np.array(s['startMm'],dtype=float);b=np.array(s['endMm'],dtype=float);length=float(np.linalg.norm(b-a))
        if length<1e-6:raise ValueError('Zero seam')
        key=tuple(sorted((tuple(np.round(a,5)),tuple(np.round(b,5)))))
        if key in seen:continue
        seen.add(key)
        direction=(b-a)/length
        count=max(1,math.ceil(length/120))
        for i in range(count):
            centre=length*(i+.5)/count;half=min(on,length/count)/2
            start=a+direction*(centre-half);end=a+direction*(centre+half)
            rows.append(dict(seam=s['id'],a=start.tolist(),b=end.tolist(),lengthMm=2*half,legMm=5))
    return rows


def capacity(stitches,force,moment):
    if not stitches:return dict(lengthMm=0,minimumLineInertiaMm3=0,centroidMm=None,stressBoundMPa=None,
        capacityReason='no eligible accessible weld edges',allowableMPa=80,passes=False)
    lengths=np.array([s['lengthMm'] for s in stitches]);centres=np.array([(np.array(s['a'])+s['b'])/2 for s in stitches]);total=float(lengths.sum())
    c=(centres*lengths[:,None]).sum(axis=0)/total;q=np.zeros((3,3));rmax=0.
    for s,mid,length in zip(stitches,centres,lengths):
        d=np.array(s['b'])-s['a'];r=mid-c;q+=length*(np.outer(r,r)+np.outer(d,d)/12)
        rmax=max(rmax,float(np.linalg.norm(np.array(s['a'])-c)),float(np.linalg.norm(np.array(s['b'])-c)))
    inertia=np.eye(3)*np.trace(q)-q;minimum=float(np.linalg.eigvalsh(inertia)[0]);throat=5/math.sqrt(2)
    stress=force/(throat*total)+(moment*rmax/(throat*minimum) if moment and minimum>1e-8 else (math.inf if moment else 0))
    return dict(lengthMm=total,minimumLineInertiaMm3=minimum,centroidMm=c.tolist(),stressBoundMPa=stress if math.isfinite(stress) else None,
                capacityReason='finite weld-group inertia' if math.isfinite(stress) else 'collinear seam group cannot resist the assumed arbitrary moment',
                allowableMPa=80,passes=math.isfinite(stress) and stress<=80)


def main():
    p=argparse.ArgumentParser();p.add_argument('--native',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    for bed in (2600,3150):
        source=args.native/f'y{bed}.json';data=json.loads(source.read_text());folder=args.output/f'y{bed}';folder.mkdir(parents=True,exist_ok=True)
        stock={m['id']:m['sourceStock'] for m in data['members']};groups=defaultdict(list)
        for seam in data['seams']:groups[tuple(sorted((stock[seam['a']['member']],stock[seam['b']['member']])))].append(seam)
        rows=[];stitches=[]
        for index,(names,seams) in enumerate(sorted(groups.items()),1):
            moment=100000 if any('Brace' in n for n in names) else 250000 if any(n.startswith(('table','rackCross')) for n in names) else 750000
            selection=None;check=None
            for on in (40,60,80,120,10000):
                selection=weld_stitches(seams,on);check=capacity(selection,5000,moment)
                if check['passes']:break
            id=f'W{index:03d}';rows.append(dict(id=id,stocks=names,forceEnvelopeN=5000,momentEnvelopeNmm=moment,nominalOnMm=on if on<10000 else None,
                extent='continuous available eligible edges' if on==10000 else f'{on} mm stitches at <=120 mm spacing',
                strengthScreen=check,excludedFlushButtEdges=sum(s['type']=='Butt' for s in seams),sequence='tack all; opposite sides/faces; centre-out; interpass cooling',
                qualification='Conservative joint-family load allocation; actual load-path/WPS shop review required'))
            for stitch in selection:stitches.append(dict(group=id,**stitch))
        def write(name,header,records):
            with (folder/name).open('w',newline='') as f:
                w=csv.writer(f,lineterminator='\n');w.writerow(header);w.writerows(records)
        write('weld-stitches.csv',['group','seam','leg_mm','length_mm','start_x','start_y','start_z','end_x','end_y','end_z'],
              [[s['group'],s['seam'],s['legMm'],s['lengthMm'],*s['a'],*s['b']] for s in stitches])
        # Alternate opposite frame quadrants; issue only one stitch per joint per
        # round, so a complete joint is never finished before its opposing bays.
        middle=np.mean([r['strengthScreen']['centroidMm'] for r in rows if r['strengthScreen']['centroidMm'] is not None],axis=0)
        queues={r['id']:[] for r in rows}
        for i,s in enumerate(stitches,1):queues[s['group']].append(i)
        quadrants=[[] for _ in range(4)]
        for r in rows:
            centre=r['strengthScreen']['centroidMm']
            if centre is not None:quadrants[int(centre[0]>=middle[0])+2*int(centre[1]>=middle[1])].append(r['id'])
        by_id={r['id']:r for r in rows}
        for q in quadrants:q.sort(key=lambda id:abs(by_id[id]['strengthScreen']['centroidMm'][1]-middle[1]))
        order=[];round_number=0
        while any(queues.values()):
            round_number+=1;active=[list(q) for q in quadrants]
            while any(active):
                for quadrant in (0,3,1,2):
                    if active[quadrant]:
                        id=active[quadrant].pop(0)
                        if queues[id]:order.append([len(order)+1,round_number,id,queues[id].pop(0),'alternate faces; cool per WPS; survey after round'])
        write('weld-sequence.csv',['run','round','group','stitch_csv_row','instruction'],order)
        result=dict(workY=bed,sourceSha256=hashlib.sha256(source.read_bytes()).hexdigest(),groups=rows,
            basis='5 mm legs, 80 MPa assumed weld-throat stress screen, F=5 kN; moments 0.1 kNm brace/0.25 table/0.75 post-frame. Uses actual selected 3D seam-line inertia; not a code/WPS certification.',
            failedGroups=[r['id'] for r in rows if not r['strengthScreen']['passes']])
        (folder/'weld-detail.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        digital=dict(preparedSeams=not data['seamDiagnostics'] and bool(data['seams']),serviceAccess=data.get('serviceAccess',{}).get('status')=='service access checks pass',
                     weldStrengthScreen=not result['failedGroups'],diyProcedure=(args.output/'DIY_ROUTE.md').exists())
        release=dict(status='GATED — engineering/shop qualification incomplete',released=False,digitalChecks=digital,
            pending=['Actual joint-load allocation and WPS/access review; resolve every failed strength-screen group',
                'Steel identification and heat-treater cycle/payload confirmation, or approved VSR procedure',
                'Selected machine epoxy TDS approves 2–6 mm layer/contact pressure and cure; witness-pour qualification',
                'Shop confirms supported beam length/travel and indexed datum-transfer tolerances',
                'Shop-release review signed against the exact prepared STEP/seam/service packet hashes'],
            buildWitnesses=['Thermal/VSR before-after-rest dimensional records','Epoxy thickness/cure and rail-seat survey','Rail parallelism/diagonals/dual-Y homing records','SR6-C measured modes/damping and motion-error recheck'],
            sourceSha256=result['sourceSha256'])
        (folder/'shop-release.json').write_text(json.dumps(release,indent=2)+'\n')
        print(bed,len(rows),'weld groups;',len(stitches),'stitches; strength-screen failures',result['failedGroups'],'digital',digital)


if __name__=='__main__':main()
