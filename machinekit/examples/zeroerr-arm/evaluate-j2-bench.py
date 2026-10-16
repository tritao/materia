#!/usr/bin/env python3
"""Reduce measured J2 loads; never energizes hardware or invents missing measurements."""
import argparse
import json
import math
from pathlib import Path


def measured_torque(bodies):
    if not bodies: raise ValueError('Measured body list is empty')
    torque=0.
    for body in bodies:
        mass=body['massKg'];lever=body['signedHorizontalCogMm']
        if mass is None or lever is None: raise ValueError('Missing measured mass/CoG: '+body['name'])
        if not math.isfinite(mass) or mass<0 or not math.isfinite(lever): raise ValueError('Invalid measured mass/lever')
        torque+=mass*9.80665*lever/1000
    return torque


def evaluate(data):
    torque=measured_torque(data['measuredBodies'])
    fixture=data['fixture'];capacity=fixture['approvedCapacityNm']
    if capacity is None or not math.isfinite(capacity) or capacity<=0: raise ValueError('Fixture capacity not reviewed')
    if not fixture['reviewed'] or not fixture['outputMountedUpstream'] or not fixture['movingHousing']:
        raise ValueError('J2 fixture convention/review not confirmed')
    if abs(torque)>capacity: raise ValueError('Measured load exceeds reviewed fixture capacity')
    rows=[]
    for sample in data['measurements']:
        observed=sample.get('externalTorqueNm')
        residual=None if observed is None else observed-torque
        rows.append(dict(timeSeconds=sample.get('timeSeconds'),externalTorqueNm=observed,
                         residualNm=residual,externalAngleRad=sample.get('externalAngleRad'),
                         motorCurrentA=sample.get('motorCurrentA'),temperatureC=sample.get('temperatureC'),
                         brakeApplied=sample.get('brakeApplied'),driveEnabled=sample.get('driveEnabled')))
    return dict(status='measurements_reduced_not_hardware_qualification',predictedGravityNm=torque,
                articleConfirmed=data['article']['confirmed'],measurements=rows,
                acceptance=data['acceptance'],qualified=False,
                caveat='External torque/angle and calibrated masses are independent evidence. Current-based torque needs delivered Kt/ratio/efficiency calibration. No acceptance inferred from null budgets.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('measurements',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=evaluate(json.loads(args.measurements.read_text()));text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
