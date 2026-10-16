#!/usr/bin/env python3
"""Retiming and sampled inverse-dynamics approval for simulation command replay."""
import hashlib
import json
from pathlib import Path
import numpy as np
from reference_dynamics import ROOT,reference_study,inverse_dynamics,load_json

BUILD=ROOT/'build/reference-control'


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def curve(points,rpm):
    values=np.array(points)
    return np.interp(abs(rpm),values[:,0],values[:,1],right=0.)


def main():
    profile=load_json(ROOT/'reference-control-profile.json');curves=load_json(ROOT/'reference-drive-curves.json')
    timed=load_json(BUILD/'timed-paths.json')
    assert timed['profileSha256']==digest(ROOT/'reference-control-profile.json')
    assert timed['sourceSha256']==digest(ROOT.parent/'reference-arms/build/arm-design/zeroerr-original-native.json')
    assert timed['timingProducerSha256']==digest(ROOT.parent/'reference-arms/FrozenReferenceControl.hx')
    study=reference_study(5,150)
    original=load_json(ROOT/'original-comparison-config.json')
    for i,ratio in enumerate(profile['nominalRatios']):
        inertia=original['catalog'][profile['families'][i]]['motorRotorInertiaGmm2']*1e-9
        study.reflected[i]=inertia*(ratio+1)**2
    catalogs=[]
    for family,ratio in zip(profile['families'],profile['nominalRatios']):
        catalogs.append(curves['curves'][family] if ratio==100 else curves['curvesByNominalRatio'][str(ratio)][family])
    outputs=[];runs=[]
    integrate=np.trapezoid if hasattr(np,'trapezoid') else np.trapz
    for path in timed['paths']:
        successful=None
        for scale in [1.,1.5,2.,3.,4.,6.,8.]:
            time=np.array([s['t']*scale for s in path['samples']]);q=np.array([s['q'] for s in path['samples']])
            v=np.array([s['velocity'] for s in path['samples']])/scale
            a=np.array([s['acceleration'] for s in path['samples']])/scale**2
            assert np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(a).all()
            window=profile['simulationJointWindowRad'];assert q.min()>=window[0] and q.max()<=window[1]
            speed_ok=(np.rad2deg(abs(v)).max(axis=0)<=np.minimum(profile['maximumVelocityDegS'],profile['requestedVelocityDegS'])+1e-6).all()
            accel_ok=(np.rad2deg(abs(a)).max(axis=0)<=profile['requestedAccelerationDegS2']+1e-6).all()
            tau=np.array([inverse_dynamics(study,x,y,z) for x,y,z in zip(q,v,a)])
            rms=np.sqrt(integrate(tau*tau,time,axis=0)/time[-1])
            curve_ok=True;peak_ok=True;gear_ok=True;average=[]
            for j,cat in enumerate(catalogs):
                rpm=abs(v[:,j])*60/(2*np.pi)
                # Non-accelerating joints use continuous curve; acceleration samples use peak region.
                caps=np.array([curve(cat['accelerationRpmNm'] if abs(acc)>1e-6 else cat['continuousRpmNm'],speed) for acc,speed in zip(a[:,j],rpm)])
                curve_ok=curve_ok and bool((abs(tau[:,j])<=caps+1e-8).all())
                peak_ok=peak_ok and bool((abs(tau[:,j])<=cat['repeatedPeakNm']).all())
                travel=integrate(rpm,time)
                avg=float((integrate(abs(tau[:,j])**3*rpm,time)/travel)**(1/3)) if travel>1e-12 else None
                average.append(avg)
                gear_ok=gear_ok and (avg is None or avg<=cat['averageLoadNm'])
            rms_ok=bool((rms<=np.array([c['ratedNm'] for c in catalogs])).all())
            if speed_ok and accel_ok and curve_ok and peak_ok and gear_ok and rms_ok:
                successful=dict(id=path['id'],timeScale=scale,durationSeconds=float(time[-1]),sampleCount=len(time),
                    peakTorqueNm=abs(tau).max(axis=0).tolist(),rmsMotionTorqueNm=rms.tolist(),speedWeightedCubicGearTorqueNm=average,
                    peakVelocityDegS=np.rad2deg(abs(v).max(axis=0)).tolist(),peakAccelerationDegS2=np.rad2deg(abs(a).max(axis=0)).tolist(),
                    curvePass=True,repeatedPeakPass=True,ratedRmsPass=True,averageGearPass=True)
                outputs.append(successful)
                runs.append(dict(id=path['id'],commands=[dict(t=float(t),q=x.tolist(),velocity=y.tolist(),torqueNm=z.tolist()) for t,x,y,z in zip(time,q,v,tau)]))
                print('Approved',path['id'],'scale',scale,'duration',round(time[-1],3),'RMS',rms.round(3).tolist(),flush=True)
                break
        if successful is None: raise ValueError('No passing timing for '+path['id']+'; investigate model/curve/duty assumptions, do not change geometry')
    pins={str(p.relative_to(ROOT)):digest(p) for p in [ROOT/'reference_dynamics.py',ROOT/'reference-baseline.json',ROOT/'reference-drive-curves.json',ROOT/'reference-control-profile.json',Path(__file__)]}
    result=dict(passed=True,simulationOnly=True,hardwareQualified=False,profileSha256=digest(ROOT/'reference-control-profile.json'),
        timedSourceSha256=digest(BUILD/'timed-paths.json'),inputs=pins,
        assumptions=['Published curve inner approximations, delivered revision unconfirmed','Whole-module housing lumps; diagonal reflected rotor inertia',
                     'Point tool/load: intrinsic inertia, friction and measured thermal duty missing','Sampled torque validation, not continuous torque proof',
                     'Four independent runs initialized at each certified path start; no unplanned transfer between runs'],
        paths=outputs,runs=runs)
    (BUILD/'approved-commands.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
    (BUILD/'validation.json').write_text(json.dumps({k:v for k,v in result.items() if k!='runs'},indent=2)+'\n')


if __name__=='__main__':main()
