"""Replay immutable J4 exports on brace-study modes; retain full modal history."""
import argparse,hashlib,itertools,json,math,sys,time
from pathlib import Path
import numpy as np
from scipy.signal import lfilter
from qualify_welds import contacts,layout_signature
from weld_model import with_foot_welds
from weld_effective import effective_stitches
from weld_report import FATIGUE_ALLOWABLE,STATIC_ALLOWABLE
from brace_study import disk

def read_history(root,tag,half=False):
    from curved_error import load_export
    name='scale-1-'+(('nominal-0-half' if tag=='nominal' else tag+'-half') if half else tag)
    meta,rows,manifest=load_export(root/name,root/(name+'.manifest.json'))
    start=meta['processStartTimeS'];steady=(np.abs(rows[:,3]-20/60)<=1e-6)&(np.max(np.abs(rows[:,4:7]),axis=1)<=1e-6)
    next_=np.flatnonzero((rows[:,0]>=start+meta['config']['dt'])&~steady)
    window=float(rows[next_[0]-1,0]-start)
    if window<.1:raise ValueError('Incomplete moving rapid/cut handover')
    return meta,rows,window

def traction_envelope(history,allowance_fields,weight,tail):
    """Simultaneous vector envelope, signed cutting/Z loads and bounded future."""
    squared=np.sum(history*history,axis=1)
    allowance=np.sum(np.abs(allowance_fields),axis=2)
    peak=design_peak=0.
    for signs in itertools.product((-1.,1.),repeat=4):
        offset=allowance_fields@np.asarray(signs)
        product=np.einsum('pct,pc->pt',history,offset)
        peak=max(peak,float(np.max(squared+2*product+np.sum(offset*offset,axis=1)[:,None])))
        design=1.35*weight+1.5*offset
        product=3*np.einsum('pct,pc->pt',history,design)
        base=2.25*squared+np.sum(design*design,axis=1)[:,None]
        design_peak=max(design_peak,float(np.max(base+np.abs(product))))
    history_range=2*math.sqrt(max(0,peak))
    tail_range=float(np.max(2*np.linalg.norm(tail+allowance,axis=1)))
    fatigue=max(history_range,tail_range)
    static=max(math.sqrt(max(0,design_peak)),float(np.max(np.linalg.norm(1.35*np.abs(weight)+1.5*(tail+allowance),axis=1))))
    interval=np.maximum(np.max(np.abs(history),axis=2),tail)+allowance
    interval_range=float(np.max(2*np.linalg.norm(interval,axis=1)))
    if fatigue>interval_range*(1+1e-10):raise ValueError('Vector fatigue envelope exceeds its component bound')
    return dict(fatigueRangeMPa=fatigue,fatigueHistoryRangeMPa=history_range,fatigueTailRangeBoundMPa=tail_range,fatigueComponentIntervalBoundMPa=interval_range,staticDesignMPa=static,fatiguePass=fatigue<=FATIGUE_ALLOWABLE,staticPass=static<=STATIC_ALLOWABLE)

def weld_response(detail,reactions,native,physical,packet,meta,rows,scale,damping,dt):
    layout=contacts(native,physical)
    if reactions.get('jointLayoutSha256')!=layout_signature(layout):raise ValueError('Weld layout/reaction signature mismatch')
    if [(c['group'],list(c['pieces'])) for c in layout]!=[(c['group'],c['pieces']) for c in reactions['contactOrder']]:raise ValueError('Weld order mismatch')
    modes=reactions['modes'];omega=np.array([2*np.pi*m['frequencyHz']*scale for m in modes]);gamma=np.array([m['gammaXY'] for m in modes]);W=np.array([m['wrench'] for m in modes]);fields=np.asarray(reactions['staticWrenches'])
    end=rows[-1,0];times=np.arange(1,math.ceil((end+.5)/dt)+1)*dt
    acc=np.column_stack([np.interp(times,rows[:,0],rows[:,i]) for i in (5,6)]);mid=np.column_stack([np.interp(times-dt/2,rows[:,0],rows[:,i]) for i in (5,6)])
    acc[times>=end]=0;mid[times-dt/2>=end]=0;q=[];amplitude=[];eq=-1000*gamma/omega[:,None]**2
    # The sampled response already includes 0.5 s after stopping. Bound only
    # the unsampled future from its final state, without discarding that history.
    tail_index=len(times)-1
    for i,w in enumerate(omega):
        wd=w*math.sqrt(1-damping*damping);decay=math.exp(-damping*w*dt);sn=math.sin(wd*dt);cs=math.cos(wd*dt)
        A=decay*(cs+damping*w*sn/wd);B=decay*sn/wd;C=-decay*w*w*sn/wd;D=decay*(cs-damping*w*sn/wd)
        forcing=mid@eq[i];den=[1.,-(A+D),A*D-B*C]
        position=lfilter([1-A,-D*(1-A)-B*C],den,forcing);velocity=lfilter([-C,C],den,forcing)
        q.append(position);amplitude.append(math.hypot(position[tail_index],(velocity[tail_index]+damping*w*position[tail_index])/wd)*math.exp(damping*w*dt))
    q=np.asarray(q);amplitude=np.asarray(amplitude);report=[]
    for index,c in enumerate(layout):
        transform=c['transform'];modal=transform@W[:,index,:].T
        wrench_modes=W[:,index,:].T
        acceleration=fields[4:6,index,:].T/3
        # Recover six force/moment histories before evaluating every throat point.
        # This is algebraically identical to filtering each throat component.
        response=transform@(wrench_modes@q+(acceleration-wrench_modes@eq)@acc.T)
        tail=(np.abs(modal)@amplitude).reshape(-1,3)
        history=response.reshape(-1,3,len(times))
        # Enumerate the four independent cutting/Z-acceleration signs at each
        # throat point. Full reversals give twice the largest vector magnitude;
        # do not combine component peaks that occur at different times.
        allowance_fields=(transform@fields[[0,1,2,6],index,:].T).reshape(-1,3,4)
        weight=(transform@fields[3,index,:]).reshape(-1,3)
        envelope=traction_envelope(history,allowance_fields,weight,tail)
        report.append(dict(group=c['group'],pieces=c['pieces'],**envelope))
    return report

def sweep(detail,j4,points,half=False,tags=None):
    from contour_response import response
    tags=tags or ['nominal','override-1p5','hold-resume-1p5','override-3','hold-resume-3']
    cases=[];sweep=list(itertools.product(np.linspace(.85,1.15,points),[.005,.0075,.01,.015,.02,.025,.03]))
    for tag in tags:
        meta,rows,window=read_history(j4,tag,half);dt=meta['config']['dt'];worst={k:dict(value=-1) for k in ['peakContourErrorMm','movingHandoverPeakMm','residualAfterStopMm']}
        for scale,damping in sweep:
            observed=response(detail['modes'],detail['staticAccelerationResponseMmPerMPerS2'],rows,meta['geometry'],float(scale),damping,dt,process_start=meta['processStartTimeS'],handover_window=window)
            for k in worst:
                if observed[k]>worst[k]['value']:worst[k]=dict(value=observed[k],frequencyScale=float(scale),dampingRatio=damping)
        cases.append(dict(history=tag,half=half,sweepCases=len(sweep),dt=dt,policy=meta['config'],durationS=meta['durationS'],worst=worst))
        print('history',tag,'max mm',max(v['value'] for v in worst.values()),flush=True)
    return cases

def main():
    p=argparse.ArgumentParser();p.add_argument('--structural',type=Path,required=True);p.add_argument('--j4',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--source',type=Path,help='Pinned replay modules when exports and source are stored separately');p.add_argument('--frequency-points',type=int,default=3);p.add_argument('--names',nargs='+');p.add_argument('--beds',nargs='+',type=int,default=[2600,3150]);p.add_argument('--poses',nargs='+',help='Optional x:y pose subset');p.add_argument('--tags',nargs='+');p.add_argument('--all-poses',action='store_true');p.add_argument('--half',action='store_true');p.add_argument('--fatigue',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    source=a.source or a.j4/'source';sys.path.insert(0,str(source));records=[];driver_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    for bed in a.beds:
        for root in sorted((a.structural/f'y{bed}').iterdir()):
            if not root.is_dir() or a.names and root.name not in a.names:continue
            for path in sorted(root.glob('*/summary.json')):
                disk();detail=json.loads(path.read_text())
                if not detail['modes']:continue
                if a.poses:
                    if (detail['pose']['x'],detail['pose']['y']) not in {tuple(map(float,value.split(':'))) for value in a.poses}:continue
                elif not a.all_poses and detail['pose']['x']!=800:continue
                name=f'y{bed}-{root.name}-x{detail["pose"]["x"]:g}-y{detail["pose"]["y"]:g}-h{detail["meshMm"]:g}'
                if a.half:name+='-half'
                target=a.output/(name+'.json')
                signature=dict(structuralSha256=hashlib.sha256(path.read_bytes()).hexdigest(),frequencyPoints=a.frequency_points,half=a.half,fatigue=a.fatigue,tags=a.tags,driverSha256=driver_sha,replaySourceSha256={n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in ['dynamic_error.py','modal_response.py','contour_response.py','curved_error.py']})
                if target.exists() and json.loads(target.read_text()).get('signature')==signature:
                    cached=json.loads(target.read_text());worst=max(v['value'] for h in cached['histories'] for v in h['worst'].values());records.append(dict(path=target.name,bed=bed,variant=root.name,pose=detail['pose'],worstMm=worst));continue
                start=time.time();report=sweep(detail,a.j4/str(bed),a.frequency_points,a.half,a.tags)
                output=dict(signature=signature,bed=bed,variant=root.name,pose=detail['pose'],meshMm=detail['meshMm'],histories=report,elapsedS=time.time()-start)
                if a.fatigue:
                    packet=json.loads((root/'packet.json').read_text());native=json.loads((root/'native.json').read_text());physical=json.loads((root/'stitches.json').read_text());joined,lines=with_foot_welds(packet,native,physical,detail['pose']);effective,_=effective_stitches(joined,lines)
                    reactions=json.loads(path.with_name('reactions.json').read_text());fatigue=[]
                    # Full sign envelope is checked separately at every structural pose.
                    # Worst coarse contour/handover/stop corners plus nominal are replayed for stress.
                    for h in report:
                        meta,rows,_=read_history(a.j4/str(bed),h['history'],a.half)
                        corners={(v['frequencyScale'],v['dampingRatio']) for v in h['worst'].values()}|{(1.,.005)}
                        for scale,damping in sorted(corners):
                            contacts_=weld_response(detail,reactions,joined,effective,packet,meta,rows,scale,damping,meta['config']['dt'])
                            fatigue.append(dict(history=h['history'],frequencyScale=scale,dampingRatio=damping,contacts=contacts_))
                    output['fatigue']=fatigue
                target.write_text(json.dumps(output,separators=(',',':'))+'\n')
                worst=max(v['value'] for h in report for v in h['worst'].values());records.append(dict(path=name+'.json',bed=bed,variant=root.name,pose=detail['pose'],worstMm=worst));(a.output/f'index-y{bed}.json').write_text(json.dumps(records,indent=2)+'\n');print(name,'worst',worst,'seconds',time.time()-start,flush=True)
if __name__=='__main__':main()
