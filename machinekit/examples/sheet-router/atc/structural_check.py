"""Focused SR-D3/weld recheck for the fixed populated ATC, closed cover.
Inputs are the retained SR6 manifests and native CAD mass delta export. The
load-bearing structure, supports and weld details are unchanged.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[2]/'bench-mill/build/fem/venv/lib/python3.12/site-packages'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'fem'))
from qualify_welds import run_case,read_stitches
from weld_model import with_foot_welds
from weld_effective import effective_stitches
from weld_dynamics import report
from weld_report import FATIGUE_ALLOWABLE,STATIC_ALLOWABLE

def fixed_fixture_clearance(parts):
 by_id={p['id']:p for p in parts};result=[]
 # The general moving-body checker intentionally ignores fixed/fixed contacts.
 # A widened rack must not bury the setter or its pedestal inside the case.
 for left in ('setter','setterPad'):
  for right in ('toolRack',*(f'fork{i}' for i in range(1,9))):
   a,b=by_id[left]['boundsMm'],by_id[right]['boundsMm']
   gap=max(max(b['min'+axis]-a['max'+axis],a['min'+axis]-b['max'+axis]) for axis in 'XYZ')
   if gap<=1e-6:raise ValueError(f'Setter fixture bounds overlap: {left}/{right}')
   result.append(dict(a=left,b=right,separatingAxisGapMm=gap))
 return result

def main():
 p=argparse.ArgumentParser()
 for key in ('inputs','mass','fabrication','ccx','sampler','output'):p.add_argument('--'+key,type=Path,required=True)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);rows=[]
 for bed in (2600,3150):
  packet=json.loads((a.inputs/f'y{bed}.json').read_text());delta=json.loads((a.mass/f'mass-y{bed}.json').read_text());old={r['id']:r for r in packet['parts']};parts={r['id']:r for r in packet['parts']}
  fixture_clearance=fixed_fixture_clearance(delta['parts'])
  for r in delta['parts']:parts[r['id']]=r
  # Four small guide/adaptor masses; their layout is an explicit allowance,
  # not a released hardware design. Model at closed-cover corners.
  cover=parts['rackCover'];c=cover['centreOfMassMm']
  for i,(dx,dy) in enumerate(((-800,-100),(-800,100),(800,-100),(800,100))):
   r=dict(cover,id=f'atcGuideAllowance{i}',massKg=delta['guideAllowanceKg']/4,centreOfMassMm=dict(x=c['x']+dx,y=c['y']+dy,z=c['z']),centroidalInertiaKgMm2World=dict(xx=125,yy=125,zz=125,xy=0,xz=0,yz=0),massSource='assumed guide/adaptor allowance',inertiaSource='assumed small isotropic guide/adaptor inertia',volumeMm3=None)
   parts[r['id']]=r
  packet['parts']=list(parts.values());mass_delta=sum(r['massKg'] for r in parts.values())-sum(r['massKg'] for r in old.values());packet['totalMassKg']+=mass_delta
  folder=a.output/f'y{bed}';folder.mkdir(exist_ok=True);(folder/'manifest.json').write_text(json.dumps(packet,indent=2)+'\n')
  pose=next(q for q in packet['poses'] if q['x']==800 and q['y']==(0 if bed==2600 else bed))
  native=json.loads((a.fabrication/f'y{bed}/native.json').read_text());native,stitches=with_foot_welds(packet,native,read_stitches(a.fabrication/f'y{bed}'),pose);stitches,_=effective_stitches(native,stitches)
  result=run_case(packet,native,stitches,pose,50,a.ccx.resolve(),folder,40)
  # The report consumes the persisted JSON wire representation (piece lists).
  result=json.loads((folder/'reactions.json').read_text())
  dynamic=report(result,native,stitches,a.sampler.resolve(),.00025);(folder/'dynamic.json').write_text(json.dumps(dynamic,indent=2)+'\n')
  k=[.3/result['cases'][i]['tipRelativeMm'][i] for i in (0,1)]
  contacts=[r for case in dynamic['cases'] for r in case['contacts']]
  row=dict(fixedFixtureClearance=fixture_clearance,workY=bed,pose=result['pose'],fixedMassDeltaKg=mass_delta,movingMassDeltaKg=0,firstModeHz=result['modes'][0]['frequencyHz'],stiffnessNPerUm=k,
   maximumContourDeviationMm=max(c['toolContourDeviationMm'] or 0 for c in dynamic['cases']),maximumResidualMm=max(c['toolResidualBoundMm'] for c in dynamic['cases']),maximumWeldRangeMPa=max(r['fatigueRangeMPa'] for r in contacts),maximumDesignStressMPa=max(r['staticPlusDynamicDesignMPa'] for r in contacts))
  row['pass']=min(k)>=2 and all(c['toolMotionBudgetPass'] for c in dynamic['cases']) and row['maximumWeldRangeMPa']<=FATIGUE_ALLOWABLE and row['maximumDesignStressMPa']<=STATIC_ALLOWABLE
  rows.append(row);print(row,flush=True)
 (a.output/'summary.json').write_text(json.dumps(dict(status='PASS' if all(r['pass'] for r in rows) else 'FAIL',checks=rows),indent=2)+'\n')
 if not all(r['pass'] for r in rows):raise ValueError('Populated ATC structural recheck failed')
if __name__=='__main__':main()
