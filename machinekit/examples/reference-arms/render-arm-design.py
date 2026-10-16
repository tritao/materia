#!/usr/bin/env python3
"""Plot original-arm load screens; no cell or commercial robot ranking."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent/'build/arm-design'
report=json.loads((ROOT/'review.json').read_text())
studies=report['originalLoads']['studies']
fig,axes=plt.subplots(1,3,figsize=(13,4.8),layout='constrained')
for ax,joint,label in zip(axes,[1,3,4],['J2 · shoulder','J4 · wrist','J5 · wrist']):
    offsets=[s['combinedCogOffsetMm'] for s in studies]
    ax.plot(offsets,[s['scenarios'][0]['gravityNm'][joint] for s in studies],'o-',label='Nominal gravity')
    ax.plot(offsets,[s['scenarios'][1]['zeroSpeedAccelerationEnvelopeNm'][joint] for s in studies],'s-',label='Conservative acceleration screen')
    ax.axhline(studies[0]['ratedTorqueNm'][joint],color='#555',linestyle='--',label='Assumed nominal-100 rated torque')
    ax.set(title=label,xlabel='Combined load CoG from flange (mm)',ylabel='Torque (Nm)',xticks=offsets)
    ax.grid(alpha=.25)
axes[0].legend(fontsize=8,loc='upper left')
fig.suptitle('Original ZeroErr arm · standalone 5 kg total flange-load screen',fontsize=14)
fig.supxlabel('Sampled hull-clear poses · assumed masses/ratios · +25% arm mass, 60°/s² and factor 1.2 for conservative screen\nZero-speed configuration estimates; no thermal duty, continuous trajectory torque or hardware qualification',fontsize=9)
for suffix in ['png','svg']:
    path=ROOT/('load-screens.'+suffix);fig.savefig(path,dpi=160);print(path)
