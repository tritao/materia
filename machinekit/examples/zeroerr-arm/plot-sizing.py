#!/usr/bin/env python3
"""Plot the engineering screen without interpreting it as hardware qualification."""
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here/'build'
if '--revised' in sys.argv: root /= 'revised'
report = json.loads((root/'sizing-report.json').read_text())
nominal, heavier = [study['joints'] for study in report['studies']]
factor = report['assumptions']['designFactor']
x = np.arange(6)
fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
for ax, field, rating, title in zip(axes,
        ['sampledGravityNm', 'sampledZeroSpeedAccelerationEnvelopeNm'],
        ['ratedTorqueNm', 'startStopPeakNm'],
        ['Gravity screen against catalog rated torque', 'Zero-speed acceleration screen against start/stop peak']):
    ax.bar(x-.23, [r[field]*factor for r in nominal], .23, label=f'Nominal mass × {factor:g}')
    ax.bar(x, [r[field]*factor for r in heavier], .23, label=f'+{100*(report["studies"][1]["structuralMassFactor"]-1):g}% arm mass × {factor:g}')
    ax.bar(x+.23, [r[rating] for r in nominal], .23, color='#777777', label='Catalog comparison')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title(title, loc='left', fontsize=11)
    ax.grid(axis='y', alpha=.2)
    ax.set_axisbelow(True)
axes[0].legend(ncol=3, fontsize=9)
axes[1].set_xticks(x, [r['joint'].upper()+' / '+r['family'] for r in nominal], fontsize=9)
a = report['assumptions']
fig.suptitle(f"{a['payloadKg']+a['toolKg']:g} kg total flange load · {a['combinedCogAlongToolNormalMm']:g} mm CoG · "
             f"{a['jointAccelerationDegS2']:g}°/s² assumed", fontsize=13)
fig.text(.5, .018, 'Preliminary exterior mass distribution. No velocity coupling, thermal qualification or full-travel clearance.',
         ha='center', fontsize=9)
fig.tight_layout(rect=(0, .04, 1, .95))
path = root/'sizing-preview.png'
fig.savefig(path, dpi=150)
print(path)
