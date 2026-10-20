"""Run both-bed weld dynamics and check time-step/modal truncation sensitivity."""
import argparse,json
from pathlib import Path
from qualify_welds import read_stitches
from weld_model import with_foot_welds
from weld_effective import effective_stitches
from weld_dynamics import report
from weld_report import FATIGUE_ALLOWABLE,STATIC_ALLOWABLE


def main():
    p=argparse.ArgumentParser()
    for name in ('results','fabrication','packets','sampler','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);checks=[]
    for bed in (2600,3150):
        folder=a.fabrication/f'y{bed}';governing=0 if bed==2600 else bed
        result=json.loads((a.results/f'y{bed}-x800-y{governing}/reactions.json').read_text())
        packet=json.loads((a.packets/f'y{bed}/manifest.json').read_text());native=json.loads((folder/'native.json').read_text())
        native,stitches=with_foot_welds(packet,native,read_stitches(folder),result['pose']);stitches,_=effective_stitches(native,stitches)
        data=[]
        for dt in (.0005,.00025):
            value=report(result,native,stitches,a.sampler,dt);data.append(value)
            (a.output/(f'dynamic-y{bed}.json' if dt==.0005 else f'dynamic-y{bed}-dt250us.json')).write_text(json.dumps(value,indent=2)+'\n')
            if not all(c['toolMotionBudgetPass'] for c in value['cases']):raise ValueError(f'Tool motion budget failed Y{bed}')
            rows=[r for c in value['cases'] for r in c['contacts']]
            if any(r['staticPlusDynamicDesignMPa']>STATIC_ALLOWABLE or r['fatigueRangeMPa']>FATIGUE_ALLOWABLE for r in rows):raise ValueError(f'Dynamic weld envelope failed Y{bed}')
        tool_delta=0.;fatigue_delta=0.;mode_tool_delta=0.;mode_fatigue_delta=0.
        for c,d in zip(data[0]['cases'],data[1]['cases']):
            for key in ('toolContourDeviationMm','toolResidualBoundMm'):
                if c[key] is not None:tool_delta=max(tool_delta,abs(c[key]-d[key]))
            fatigue_delta=max(fatigue_delta,max(abs(x['fatigueRangeMPa']-y['fatigueRangeMPa']) for x,y in zip(c['contacts'],d['contacts'])))
        for c,d in zip(data[1]['cases'][:6],data[1]['cases'][6:]):
            for key in ('toolContourDeviationMm','toolResidualBoundMm'):
                if c[key] is not None:mode_tool_delta=max(mode_tool_delta,abs(c[key]-d[key]))
            mode_fatigue_delta=max(mode_fatigue_delta,max(abs(x['fatigueRangeMPa']-y['fatigueRangeMPa']) for x,y in zip(c['contacts'],d['contacts'])))
        if max(tool_delta,mode_tool_delta)>.0005 or max(fatigue_delta,mode_fatigue_delta)>.1:raise ValueError(f'Time-step/modal convergence needs investigation Y{bed}')
        row=dict(workY=bed,timeStepToolDeltaMm=tool_delta,timeStepFatigueDeltaMPa=fatigue_delta,
            mode20To40ToolDeltaMm=mode_tool_delta,mode20To40FatigueDeltaMPa=mode_fatigue_delta,
            maximumContourDeviationMm=max(c['toolContourDeviationMm'] or 0 for d in data for c in d['cases']),
            maximumResidualMm=max(c['toolResidualBoundMm'] for d in data for c in d['cases']),
            maximumDynamicWeldRangeMPa=max(r['fatigueRangeMPa'] for d in data for c in d['cases'] for r in c['contacts']))
        checks.append(row);print(row,flush=True)
    (a.output/'motion-convergence.json').write_text(json.dumps(dict(status='PASS',checks=checks),indent=2)+'\n')


if __name__=='__main__':main()
