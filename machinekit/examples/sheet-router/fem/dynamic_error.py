"""Tool/workpiece motion error from native planner samples and FE modal residues.
No servo-loop or cutting-force dynamics are included. Frequencies/damping are not measured.
"""
import argparse
from bisect import bisect_right
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import subprocess
from modes import displacements
from study import make_model


def modal_basis(packet,pose,dat,static):
    model=make_model(packet,pose,50);model.deck([])
    masses=list(model.mass_loads)
    for _,conn,row in model.elements:
        _,rho=row['material']
        if not rho:continue
        _,w,h,*walls=row['sec'];area=w*h-(w-2*walls[0])*(h-2*walls[0]);length=math.dist(model.nodes[conn[0]],model.nodes[conn[2]])
        for n,weight in zip(conn,(1/6,4/6,1/6)):masses.append((n,area*length*rho*weight,'y' if row['group']=='gantry' else 'frame'))
    from study import frequencies
    hz=frequencies(dat);blocks=re.split(r'E I G E N V A L U E\s+N U M B E R\s+\d+',dat)[1:];rows=[]
    for index,(f,block) in enumerate(zip(hz,blocks),1):
        u=displacements(block);modal_mass=sum(m*sum(v*v for v in u[n]) for n,m,_ in masses)
        output=[u[model.query_nodes['tip']][i]-u[model.query_nodes['bed']][i] for i in range(2)]
        excitation=[sum(m*u[n][i] for n,m,carrier in masses if carrier in (('x','z') if i==0 else ('x','y','z'))) for i in range(2)]
        residues=[[output[o]*excitation[i]/modal_mass for i in range(2)] for o in range(2)]
        rows.append(dict(mode=index,frequencyHz=f,toolRelativeEigenvectorXY=output,generalizedMassKg=modal_mass,accelerationResidueXY=residues))
    if len(rows)!=packet['modeCount']:raise ValueError('Incomplete mode basis')
    # Static D'Alembert loads from the same FE packet: 3 m/s² X/Y acceleration cases.
    equilibrium=[[static['cases'][4+i]['toolRelativeMm'][o]/3 for i in range(2)] for o in range(2)]
    return rows,equilibrium


def profile(sampler,backend,distance,vmax,v0,v1,dt):
    raw=subprocess.check_output([str(sampler.resolve()),backend,str(distance),str(vmax),str(v0),str(v1),str(dt)],text=True)
    raw=raw[raw.index('# duration '):];duration=float(raw.splitlines()[0].split()[-1]);rows=[[float(v) for v in r] for r in list(csv.reader(io.StringIO(raw)))[2:]]
    if max(abs(r[3]) for r in rows)>3.00001:raise ValueError('Planner acceleration cap exceeded')
    if max(abs(r[2]) for r in rows)>vmax+1e-6:raise ValueError('Planner velocity cap exceeded')
    if backend=='ruckig' and max(abs(r[4]) for r in rows)>20.00001:raise ValueError('Planner jerk cap exceeded')
    return dict(duration=duration,rows=rows,times=[r[0] for r in rows],distance=distance,v0=rows[0][2],v1=rows[-1][2])


def sample(p,t):
    if t<0:return [p['v0']*t,p['v0'],0.]
    if t>=p['duration']:return [p['distance']+p['v1']*(t-p['duration']),p['v1'],0.]
    i=max(0,min(len(p['rows'])-2,bisect_right(p['times'],t)-1));a,b=p['rows'][i:i+2];q=(t-a[0])/(b[0]-a[0])
    return [a[k]+q*(b[k]-a[k]) for k in (1,2,3)]


def shaper(kind,hz,zeta):
    if kind=='none':return [(0.,1.)]
    delay=.5/(hz*math.sqrt(1-zeta*zeta));k=math.exp(-math.pi*zeta/math.sqrt(1-zeta*zeta))
    zv=[(0.,1/(1+k)),(delay,k/(1+k))]
    if kind=='ZV':return zv
    if kind=='ZVD':return [(0.,1/(1+k)**2),(delay,2*k/(1+k)**2),(2*delay,k*k/(1+k)**2)]
    raise ValueError(kind)


def shaped(p,t,pulses):
    states=[(sample(p,t-delay),weight) for delay,weight in pulses]
    return [sum(s[k]*w for s,w in states) for k in range(3)]


def distance_to_path(x,y,points):
    best=math.inf
    for a,b in zip(points,points[1:]):
        dx=b[0]-a[0];dy=b[1]-a[1];t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
        best=min(best,math.hypot(x-a[0]-t*dx,y-a[1]-t*dy))
    return best


def run_response(basis,equilibrium,profiles,kind,scale,zeta,dt,count=40,detune=0.,continuous=False,trace=False):
    tuning=26.05352;pulses=shaper(kind,tuning,.01);tail=pulses[-1][0]
    points=[(0.,0.),(.3,0.),(.3,.18),(.18,.18),(.18,.10),(0.,.10),(0.,0.)]
    times=[0.]
    for p in profiles:times.append(times[-1]+p['duration']+tail)
    contour_end=times[-1]
    def command(t):
        if t<0:return (0.,0.),(0.,0.)
        edge=min(len(profiles)-1,max(0,bisect_right(times,t)-1));a,b=points[edge:edge+2];p=profiles[edge]
        q,_,accel=shaped(p,t-times[edge],pulses)
        if edge==len(profiles)-1 and t>=contour_end:q=p['distance'];accel=0.
        direction=((b[0]-a[0])/p['distance'],(b[1]-a[1])/p['distance'])
        return (1000*(a[0]+direction[0]*q),1000*(a[1]+direction[1]*q)),(direction[0]*accel,direction[1]*accel)
    if len(profiles)==1:
        p=profiles[0];contour_end=p['duration']+tail
        def command(t):
            q,_,a=shaped(p,t,pulses);return (0.,1000*q),(0.,a)
    active=basis[:count];state=[];static_modal=[[0.,0.],[0.,0.]]
    for row in active:
        w=2*math.pi*row['frequencyHz']*scale*(1+detune);wd=w*math.sqrt(1-zeta*zeta);decay=math.exp(-zeta*w*dt);sn=math.sin(wd*dt);cs=math.cos(wd*dt)
        state.append([0.,0.,0.,0.,decay*(cs+zeta*w*sn/wd),decay*sn/wd,-decay*w*w*sn/wd,decay*(cs-zeta*w*sn/wd),w*w,row['accelerationResidueXY']])
        for o in range(2):
            for i in range(2):static_modal[o][i]+=-1000*row['accelerationResidueXY'][o][i]/(w*w)
    residual=[[equilibrium[o][i]/(scale*(1+detune))**2-static_modal[o][i] for i in range(2)] for o in range(2)]
    peak_contour=peak_elastic=peak_after=0.;trace_rows=[];post=[]
    steps=math.ceil((contour_end+.5)/dt)
    for j in range(steps):
        t=(j+.5)*dt;_,accel=command(t)
        for s in state:
            ex,vx,ey,vy,A,B,C,D,w2,res=s
            fx=-1000*(res[0][0]*accel[0]+res[0][1]*accel[1])/w2;fy=-1000*(res[1][0]*accel[0]+res[1][1]*accel[1])/w2
            s[0]=A*ex+B*vx+(1-A)*fx;s[1]=C*ex+D*vx-C*fx
            s[2]=A*ey+B*vy+(1-A)*fy;s[3]=C*ey+D*vy-C*fy
        now=(j+1)*dt;q,a=command(now)
        error=[sum(s[0 if o==0 else 2] for s in state)+sum(residual[o][i]*a[i] for i in range(2)) for o in range(2)]
        elastic=math.hypot(*error);peak_elastic=max(peak_elastic,elastic)
        if len(profiles)>1:
            deviation=distance_to_path((q[0]+error[0])/1000,(q[1]+error[1])/1000,points)*1000
            peak_contour=max(peak_contour,deviation)
        if now>=contour_end:peak_after=max(peak_after,elastic);post.append((now-contour_end,elastic))
        if trace and j%4==0:trace_rows.append([now,q[0],q[1],error[0],error[1]])
    settling=max((t for t,e in post if e>.05),default=0.)
    return dict(shaper=kind,modeCount=count,frequencyScale=scale,dampingRatio=zeta,detuneFraction=detune,addedTimePerMoveMs=tail*1000,
        contourDurationS=contour_end,peakContourErrorMm=peak_contour if len(profiles)>1 else None,peakElasticFollowingMm=peak_elastic,
        residualPeakAfterTransitionMm=peak_after,settlingTo005MmAfterTransitionS=settling,staticResidualMmPerMPerS2=residual,trace=trace_rows)


def main():
    p=argparse.ArgumentParser();p.add_argument('--packets',required=True,type=Path);p.add_argument('--extended',required=True,type=Path);p.add_argument('--sampler',required=True,type=Path);p.add_argument('--output',required=True,type=Path);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    packet=json.loads((args.packets/'y3150/manifest.json').read_text());packet['modeCount']=40;packet['floorSupport']='anchored';pose=packet['poses'][4]
    extended=json.loads((args.extended/'extended-modes.json').read_text());static=next(r for r in extended['rows'] if r['workY']==3150)['results'][1]
    dat=args.extended/'y3150-h50/frame.dat';basis,equilibrium=modal_basis(packet,pose,dat.read_text(),static)
    cases=[];dt=.0005
    lengths=[.3,.18,.12,.08,.18,.10]
    for backend in ('toppra','ruckig'):
        contour=[profile(args.sampler,backend,length,20/60,0,0,dt) for length in lengths]
        rapid_test=profile(args.sampler,backend,.5,40/60,0,0,dt);actual_rapid=max(r[2] for r in rapid_test['rows']);actual_cut=max(r[2] for r in contour[0]['rows'])
        rapid=[profile(args.sampler,backend,.3,40/60,actual_rapid,actual_cut,dt)];actual_rapid=rapid[0]['v0'];actual_cut=rapid[0]['v1']
        for kind,scale in [('none',1.),('ZV',1.),('ZVD',1.),('none',40/26.05352)]:
            row=dict(backend=backend,actualRapidMPerMin=actual_rapid*60,actualCutMPerMin=actual_cut*60,structure=('current' if scale==1 else 'uniform stiffness scaled to first gantry 40 Hz'),shaper=kind)
            row['contour']=run_response(basis,equilibrium,contour,kind,scale,.01,dt,trace=True)
            row['rapidToCut']=run_response(basis,equilibrium,rapid,kind,scale,.01,dt,trace=True)
            row['truncation']=[dict(modes=n,contour=run_response(basis,equilibrium,contour,kind,scale,.01,dt,n),rapidToCut=run_response(basis,equilibrium,rapid,kind,scale,.01,dt,n)) for n in (3,6,20)]
            row['uncertainty']=[dict(damping=z,detune=d,contour=run_response(basis,equilibrium,contour,kind,scale,z,dt,detune=d),rapidToCut=run_response(basis,equilibrium,rapid,kind,scale,z,dt,detune=d)) for z,d in ((.005,0),(.02,0),(.05,0),(.01,-.1),(.01,.1),(.01,-.15),(.01,.15))]
            fine_contour=[profile(args.sampler,backend,length,20/60,0,0,dt/2) for length in lengths];fine_rapid=[profile(args.sampler,backend,.3,40/60,actual_rapid,actual_cut,dt/2)]
            row['timeStepCheck']=dict(contour=run_response(basis,equilibrium,fine_contour,kind,scale,.01,dt/2),rapidToCut=run_response(basis,equilibrium,fine_rapid,kind,scale,.01,dt/2))
            cases.append(row);print(backend,row['structure'],kind,row['contour']['peakContourErrorMm'],row['contour']['peakElasticFollowingMm'],row['rapidToCut']['residualPeakAfterTransitionMm'],flush=True)
    files=[Path(__file__),dat,args.sampler,Path('motionkit/robot/haxe/motionkit/robot/PlanningLimits.hx'),Path('motionkit/robot/haxe/motionkit/robot/ProgramCompiler.hx')]
    identity={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    result=dict(status='Conditional open-loop elastic motion-error screen; SR-D3 unchanged',budgetMm=.05,budgetSource='Review-proposed engineering allocation for motion-induced finishing contour deviation, not a universal wood/plastics tolerance or total part accuracy claim',
        assumptions=['Damping 1% unmeasured; 0.5/2/5% and ±10% frequency sensitivities included','Perfect rigid command following; no servo-loop, backlash, cutting force or tool compliance dynamics','Native TOPPRA contours have acceleration steps and no certified jerk bound; native Ruckig comparison uses PlanningLimits default 20 m/s³','Exact-stop L-shaped 300x180 mm nested-panel perimeter with 180x80 notch; each shaped edge drains before the next edge','300 mm Y rapid transition from 40 to 20 m/min; residual measured for 0.5 s after shaper tail','Hypothetical 40 Hz case scales all stiffness by (40/26.05352)^2, unchanged mass and mode shapes; residual flexibility scaled consistently'],
        modalBasis=basis,staticAccelerationResponseMmPerMPerS2=equilibrium,cases=cases,identity=identity)
    (args.output/'dynamic-error.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
