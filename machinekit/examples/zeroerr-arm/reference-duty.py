#!/usr/bin/env python3
"""A stated five-kilogram motion duty; separate from braked static holding."""
import json
import numpy as np
from reference_dynamics import reference_study,inverse_dynamics,terms,load_json,ROOT


def curve(points,rpm):
    return np.interp(np.abs(rpm),np.array(points)[:,0],np.array(points)[:,1],left=points[0][1],right=0.)


def run():
    frozen=load_json(ROOT/'reference-baseline.json');curves=load_json(ROOT/'reference-drive-curves.json')
    study=reference_study(5,150);time=[];torque=[];speeds=[];accelerations=[]
    duration=4.;samples=81
    for leg in range(2):
        start,end=(0,np.deg2rad(20)) if leg==0 else (np.deg2rad(20),0)
        for u in np.linspace(0,1,samples):
            q=np.zeros(6);v=q.copy();a=q.copy()
            q[0]=start+(end-start)*(10*u**3-15*u**4+6*u**5)
            v[0]=(end-start)*(30*u**2-60*u**3+30*u**4)/duration
            a[0]=(end-start)*(60*u-180*u**2+120*u**3)/duration**2
            time.append(leg*duration+u*duration);torque.append(inverse_dynamics(study,q,v,a));speeds.append(v);accelerations.append(a)
    time=np.array(time);tau=np.array(torque);velocity=np.array(speeds);accel=np.array(accelerations)
    dwell=12.;total=2*duration+dwell
    rated=np.array([curves['curves'][f]['ratedNm'] for f in frozen['modules']])
    # Brake holds dwell with servo disabled: zero modeled servo torque during dwell.
    rms=np.sqrt(np.trapezoid(tau*tau,time,axis=0)/total) if hasattr(np,'trapezoid') else np.sqrt(np.trapz(tau*tau,time,axis=0)/total)
    gear_average=[];continuous=[];peak=[]
    for i,family in enumerate(frozen['modules']):
        cat=curves['curves'][family];rpm=np.abs(velocity[:,i])*60/(2*np.pi)
        integrate=np.trapezoid if hasattr(np,'trapezoid') else np.trapz
        travel=integrate(rpm,time)
        gear_average.append(float((integrate(np.abs(tau[:,i])**3*rpm,time)/travel)**(1/3)) if travel>1e-12 else 0.)
        continuous.append(bool((np.abs(tau[:,i])<=curve(cat['continuousRpmNm'],rpm)).all()))
        peak.append(bool((np.abs(tau[:,i])<=curve(cat['accelerationRpmNm'],rpm)).all()))
    output=dict(totalFlangeLoadKg=5,combinedCogMm=150,motion='upright original arm; J1 0 to 20 degrees and back; other joints zero',
        movingSeconds=8,brakedDwellSeconds=dwell,totalSeconds=total,
        peakTorqueNm=np.abs(tau).max(axis=0).tolist(),rmsServoTorqueOverWholeCycleNm=rms.tolist(),
        rmsBelowRated=bool((rms<=rated).all()),continuousCurvePass=all(continuous),accelerationCurvePass=all(peak),
        gearSpeedWeightedCubicAverageNm=gear_average,peakSpeedDegS=np.rad2deg(np.abs(velocity).max(axis=0)).tolist(),
        peakAccelerationDegS2=np.rad2deg(np.abs(accel).max(axis=0)).tolist(),
        qualification='Explicit simulation duty only. Braked dwell has zero modeled servo torque; actual enable/brake sequence and thermal curves unconfirmed. No physical test. Does not claim every 5 kg trajectory passes.')
    build=ROOT/'build/reference-reconciliation';build.mkdir(parents=True,exist_ok=True)
    (build/'duty.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))
    return output


if __name__=='__main__':run()
