"""DIY foot-joint screen at each bed's modal and static governing poses."""
import argparse,json
from copy import deepcopy
from pathlib import Path
from study import solve
from sensitivity import first_gantry


def main():
 p=argparse.ArgumentParser();p.add_argument('--machined-contact',action='store_true');p.add_argument('--packets',type=Path,required=True);p.add_argument('--ccx',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 data=json.loads((a.packets/'results.json').read_text());summary=json.loads((a.packets/'summary.json').read_text());rows=[]
 for length in (2600,3150):
  packet=json.loads((a.packets/f'y{length}/manifest.json').read_text());packet['floorSupport']='anchored';packet['beamFootJointWelded']=False
  if a.machined_contact:packet['interfaces'].update(beamFootMachinedContactNPerMm=1500000.,beamFootDowelShearNPerMm=200000.)
  v=next(r for r in summary['variants'] if r['workY']==length)
  poses=[('modal',v['lowestGantryDominantMode']['pose']),('staticY',v['worstCutting'][1]['pose']),('staticX',v['worstCutting'][0]['pose'])]
  for label,pose in poses:
   result=solve(packet,pose,50,a.ccx.resolve(),a.output/f'y{length}-{label}')
   baseline=next(r for r in data['results'] if r['workY']==length and r['meshMm']==50 and r['pose']==pose)
   row=dict(workY=length,case=label,result=result,baselineXY=[1/baseline['cases'][i]['complianceUmPerN'][i] for i in (0,1)],boltedXY=[1/result['cases'][i]['complianceUmPerN'][i] for i in (0,1)],baselineGantryHz=first_gantry(baseline)['frequencyHz'],boltedGantryHz=first_gantry(result)['frequencyHz'])
   rows.append(row);(a.output/'feet.json').write_text(json.dumps(dict(status='Assumed lumped bolted-joint compliance; no supplier qualification',rows=rows),indent=2)+'\n');print(length,label,row['baselineXY'],row['boltedXY'],row['baselineGantryHz'],row['boltedGantryHz'],flush=True)

if __name__=='__main__':main()
