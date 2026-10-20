"""Modal weld-force history at the real acceleration/jerk/feed limits.

Uses native prepared-contact mode shapes and spring reactions. The omitted-mode
static residual matches the FE acceleration response. Post-transition
modal amplitude bounds include every later phase, not only a sampled tail.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from qualify_welds import contacts,read_stitches,layout_signature
from weld_effective import effective_stitches
from weld_model import with_foot_welds
from dynamic_error import profile,sample
from weld_report import STATIC_ALLOWABLE,FATIGUE_ALLOWABLE,FATIGUE_CYCLES


def histories(basis,profiles,directions,zeta,dt,count):
    basis=basis[:count];omega=2*np.pi*np.array([r['frequencyHz'] for r in basis]);gamma=np.array([r['gammaXY'] for r in basis]);wd=omega*np.sqrt(1-zeta*zeta)
    decay=np.exp(-zeta*omega*dt);sn=np.sin(wd*dt);cs=np.cos(wd*dt)
    A=decay*(cs+zeta*omega*sn/wd);B=decay*sn/wd;C=-decay*omega**2*sn/wd;D=decay*(cs-zeta*omega*sn/wd)
    times=np.r_[0,np.cumsum([p['duration'] for p in profiles])];end=times[-1];steps=math.ceil((end+.5)/dt)
    state=np.zeros(count);velocity=np.zeros(count);q=np.empty((count,steps));acc=np.empty((2,steps))
    tail_amplitude=None
    for j in range(steps):
        t=(j+.5)*dt;edge=min(len(profiles)-1,max(0,int(np.searchsorted(times,t,side='right'))-1))
        a=np.array(directions[edge])*sample(profiles[edge],t-times[edge])[2] if t<end else np.zeros(2)
        equilibrium=-1000*(gamma@a)/omega**2
        previous=state;state=A*state+B*velocity+(1-A)*equilibrium;velocity=C*previous+D*velocity-C*equilibrium
        q[:,j]=state;acc[:,j]=a
        if tail_amplitude is None and (j+1)*dt>=end:tail_amplitude=np.sqrt(state**2+((velocity+zeta*omega*state)/wd)**2)*np.exp(zeta*omega*dt)
    return q,acc,tail_amplitude,-1000*gamma/omega[:,None]**2


def contour_deviation(profiles,axes,tool,dt):
    points=np.vstack((np.zeros(2),np.cumsum([np.array(a)*p['distance']*1000 for a,p in zip(axes,profiles)],axis=0)))
    times=np.r_[0,np.cumsum([p['duration'] for p in profiles])];command=np.empty_like(tool)
    for i in range(tool.shape[1]):
        now=(i+1)*dt;edge=min(len(profiles)-1,max(0,int(np.searchsorted(times,now,side='right'))-1))
        command[:,i]=points[edge]+np.array(axes[edge])*sample(profiles[edge],now-times[edge])[0]*1000
    actual=(command+tool).T;distance=np.full(len(actual),np.inf)
    for a,b in zip(points,points[1:]):
        d=b-a;t=np.clip((actual-a)@d/(d@d),0,1)
        distance=np.minimum(distance,np.linalg.norm(actual-a-t[:,None]*d,axis=1))
    return float(np.max(distance))


def report(result,native,stitches,sampler,dt=.0005):
    layout=contacts(native,stitches);fields=np.array(result['staticWrenches']);modes=result['modes']
    if [(c['group'],list(c['pieces'])) for c in layout]!=[(c['group'],c['pieces']) for c in result['contactOrder']]:raise ValueError('Reaction/contact ordering mismatch')
    if result.get('jointLayoutSha256')!=layout_signature(layout):raise ValueError('Reaction/weld detail signature mismatch')
    contour=[profile(sampler,'ruckig',d,20/60,0,0,dt) for d in (.3,.18,.12,.08,.18,.10)]
    directions=[(1,0),(0,1),(-1,0),(0,-1),(-1,0),(0,-1)]
    # Ruckig profiles use the configured 3 m/s² and 20 m/s³ limits.
    programs=[('contour',contour,directions),('rapid-to-cut',[profile(sampler,'ruckig',.3,40/60,40/60,20/60,dt)],[(0,1)]),
              ('rapid-stop',[profile(sampler,'ruckig',.5,40/60,40/60,0,dt)],[(0,1)])]
    observed=[]
    for count in (20,40):
        if len(modes)<count:raise ValueError('Incomplete weld mode basis')
        W=np.array([r['wrench'] for r in modes[:count]])
        for zeta in (.005,.01):
            for name,profiles,axes in programs:
                q,acc,amplitude,equilibrium=histories(modes,profiles,axes,zeta,dt,count)
                output=np.array([r['toolRelativeEigenvectorXY'] for r in modes[:count]]).T
                static=np.array([r['tipRelativeMm'][:2] for r in result['cases'][4:6]]).T/3
                tool=output@q+(static-output@equilibrium)@acc
                tool_peak=float(np.max(np.linalg.norm(tool,axis=0)))
                residual_bound=float(np.linalg.norm(np.abs(output)@amplitude))
                deviation=contour_deviation(profiles,axes,tool,dt) if name=='contour' else None
                rows=[]
                for i,c in enumerate(layout):
                    transform=c['transform'];modal=transform@W[:,i,:].T
                    acceleration=transform@fields[4:6,i,:].T/3
                    residual=acceleration-modal@equilibrium
                    response=modal@q+residual@acc
                    bound=np.maximum(np.max(np.abs(response),axis=1),np.abs(modal)@amplitude).reshape(-1,3)
                    cutting=np.sum(np.abs(transform@fields[:3,i,:].T),axis=1).reshape(-1,3)
                    weight=(transform@fields[3,i,:]).reshape(-1,3)
                    variable=bound+cutting
                    design=np.linalg.norm(1.35*np.abs(weight)+1.5*variable,axis=1)
                    fatigue=2*np.linalg.norm(variable,axis=1)
                    motionW=W[:,i,:].T@q+(fields[4:6,i,:].T/3-W[:,i,:].T@equilibrium)@acc
                    wrenchBound=np.maximum(np.max(np.abs(motionW),axis=1),np.abs(W[:,i,:].T)@amplitude)
                    rows.append(dict(group=c['group'],pieces=c['pieces'],staticPlusDynamicDesignMPa=float(np.max(design)),
                        fatigueRangeMPa=float(np.max(fatigue)),motionReactionComponentBoundN_Nmm=wrenchBound.tolist()))
                observed.append(dict(program=name,damping=zeta,modeCount=count,toolElasticPeakMm=tool_peak,toolContourDeviationMm=deviation,toolResidualBoundMm=residual_bound,
                    toolMotionBudgetPass=(deviation is None or deviation<=.05) and residual_bound<=.05,contacts=rows))
    return dict(status='Native weld reactions, conservative vector stress/fatigue envelope; 100-million-reversal user duty; material/WPS shop confirmation required',
        pose=result['pose'],modeFrequenciesHz=[m['frequencyHz'] for m in modes],dtS=dt,limits=dict(accelerationMPerS2=3,jerkMPerS3=20,contourMPerMin=20,rapidMPerMin=40),
        staticAllowableMPa=STATIC_ALLOWABLE,fatigueCategoryMPa=36,fatigueCycles=FATIGUE_CYCLES,fatigueAllowableMPa=FATIGUE_ALLOWABLE,cases=observed)


def main():
    p=argparse.ArgumentParser();p.add_argument('--result',type=Path,required=True);p.add_argument('--fabrication',type=Path,required=True);p.add_argument('--sampler',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--packet',type=Path,required=True);p.add_argument('--dt',type=float,default=.0005);a=p.parse_args()
    result=json.loads(a.result.read_text());native=json.loads((a.fabrication/'native.json').read_text());native,physical=with_foot_welds(json.loads(a.packet.read_text()),native,read_stitches(a.fabrication),result['pose']);stitches,_=effective_stitches(native,physical)
    if a.dt not in (.0005,.00025):raise ValueError('Use the checked 0.5 or 0.25 ms integration step')
    data=report(result,native,stitches,a.sampler,a.dt)
    a.output.write_text(json.dumps(data,indent=2)+'\n')
    for count in (20,40):
        rows=[r for c in data['cases'] if c['modeCount']==count for r in c['contacts']]
        print(count,'modes: static failures',sorted({r['group'] for r in rows if r['staticPlusDynamicDesignMPa']>STATIC_ALLOWABLE}),
              'fatigue failures',sorted({r['group'] for r in rows if r['fatigueRangeMPa']>FATIGUE_ALLOWABLE}),flush=True)

if __name__=='__main__':main()
