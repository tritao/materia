#!/usr/bin/env python3
"""Render the standalone actuator trade study, with source and qualification limits."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parent
BUILD=ROOT/'build/arm-options'
report=json.loads((BUILD/'comparison.json').read_text())
labels={
    'original':'Original 110T / 70T', 'wrist80_only':'80T at J4/J5 only',
    'shoulder142_r50':'142T50 + 80T wrists', 'shoulder142_r80':'142T80 + 80T wrists',
    'shoulder142_r100':'142T100 + 80T wrists',
    'shoulder142_all_wrist80':'142T100 + all three 80T wrists',
    'shoulder170_r50':'170I50 + 80T wrists', 'shoulder170_r80':'170I80 + 80T wrists'}
fig,axes=plt.subplots(1,2,figsize=(15,7.8),sharey=True,gridspec_kw={'wspace':.12})
y=np.arange(len(report['options']))
for case,shift,color,label in [('drawing_materials',-.16,'#2878a6','Drawing masses + material corrections'),
                                ('drawing_plus25percent',.16,'#d47743','Arm masses +25% sensitivity')]:
    domain=[];paths=[]
    for option in report['options']:
        row=next(c for c in option['cases'] if c['massScenario']==case and c['combinedCogOffsetMm']==150)
        screen=next(s for s in row['screens'] if s['accelerationDegS2']==60)
        domain.append(max(screen['maxDemandOverRatedByJoint']))
        paths.append(max(p['maxDemandOverRated'] for p in screen['paths']))
    for ax,values in zip(axes,[domain,paths]):
        ax.barh(y+shift,values,height=.29,color=color,label=label)
        for yy,value in zip(y+shift,values): ax.text(value+.02,yy,f'{value:.2f}',va='center',fontsize=9)
for ax,title in zip(axes,['118 original-arm clear configurations','Samples along four standalone motions']):
    ax.axvline(1,color='#333333',linestyle='--',linewidth=1.2)
    ax.set_xlim(0,2.8);ax.set_title(title,fontsize=12,pad=14)
    ax.set_xlabel('Screened demand / assumed rated torque')
    ax.grid(axis='x',alpha=.18);ax.set_axisbelow(True)
    ax.spines[['top','right']].set_visible(False)
axes[0].set_yticks(y,[f"{labels[o['id']]}\n{o['estimatedArmMassKg']:.1f} kg · J2 peak {o['publishedPeakSpeedDegS'][1]:g}°/s" for o in report['options']],fontsize=10)
axes[0].invert_yaxis()
fig.suptitle('Shoulder and wrist sizing trade-off',fontsize=18,y=.97)
fig.text(.5,.915,'5 kg total flange load · 150 mm combined CoG · 60°/s² zero-speed bound · design factor 1.2',ha='center',fontsize=11)
axes[1].legend(loc='lower right',fontsize=9,framealpha=.95)
fig.text(.04,.025,'Values below 1 pass this sampled load screen only. Resized options use original axes and estimated mass distribution.\nNew housings, adapters, self-clearance, tool inertia, torque–speed curves and thermal duty remain unqualified.',fontsize=10,color='#444444')
fig.subplots_adjust(left=.27,right=.98,top=.85,bottom=.14)
for suffix in ['png','svg']:
    path=BUILD/('options-comparison.'+suffix);fig.savefig(path,dpi=160);print(path)
