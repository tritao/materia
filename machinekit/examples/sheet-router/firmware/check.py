#!/usr/bin/env python3
"""Check resource allocation against a manually reviewed manufacturer table subset."""
import copy,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[3]
def validate(d,e,schematic):
 pins=d['pins'];assert len({p['pin'] for p in pins})==len(pins),'physical pin collision'
 assert len({p['signal'] for p in pins})==len(pins),'duplicate signal'
 available={f'P{port}{i}' for port in 'ABC' for i in range(16)}|{'PD2','PF0','PF1'}
 for p in pins:
  assert p['pin'] in available,'pin absent from LQFP64'
  fn=p['function']
  if p['af'] is not None:assert e['alternateFunctions'].get(p['pin'],{}).get(str(p['af']))==fn,'unsupported alternate function'
  elif fn.startswith('EXTI'):assert fn=='EXTI'+p['pin'][2:],'wrong EXTI line'
  elif fn!='GPIO_OUT':assert e['analog'].get(p['pin'])==fn,'wrong analog assignment'
 exti=[p['function'] for p in pins if p['function'].startswith('EXTI')];assert len(set(exti))==14,'EXTI line collision'
 steps=[p['function'].split('_')[0] for p in pins if p['signal'].endswith('.step')]
 enc=[p['function'].split('_')[0] for p in pins if p['signal'].endswith('encoderA')]
 assert len(steps)==len(set(steps))==5 and len(enc)==len(set(enc))==5
 assert not set(steps)&set(enc),'encoder/step timer collision'
 assert set(enc)==set(e['encoderTimers'])
 assert len({(x['controller'],x['channel']) for x in d['dma']})==len(d['dma'])
 assert len(d['dma'])<=16
 for x in d['dma']:
  assert x['controller'] in ['DMA1','DMA2'] and 1<=x['channel']<=8
  assert e['dmaRequests'].get(x['signal'])==x['request'],'wrong DMA request'
 assert d['timerClockHz']//(d['prescaler']+1)==d['counterHz']==10_000_000
 assert d['maxStepHzPerAxis']==200_000
 mapping=d['channels'];assert len(mapping)==len(schematic['channels'])
 assert len({x['devicePin'] for x in mapping})==len(mapping),'device pin collision'
 assert {(x['channel'],x['carrierPin'],x['terminal']) for x in mapping}=={(x['id'],x['boardPin'],x['terminal']) for x in schematic['channels']},'SR17 reverse map mismatch'
 for row in schematic['channels']:
  target=next(x['devicePin'] for x in mapping if x['channel']==row['id'])
  if row['kind'] in ['step','direction','encoder','fast-input','analog']:
   signal='spindle.speed' if row['kind']=='analog' else row['id']
   assert target==next(p['pin'] for p in pins if p['signal']==signal),'channel/function pin mismatch'
  else:assert re.fullmatch(r'U(IN[1-3]|OUT[1-2])\.GP([0-9]|1[0-5])',target),'invalid expander port'
 profile=(REPO/'robotkit/device_protocol/boards/nucleo-g474re/src/router_profile.rs').read_text()
 declared={(a,b,int(c)) for a,b,c in re.findall(r'Pin\s*\{\s*signal:\s*"([^"]+)",\s*pin:\s*"([^"]+)",\s*af:\s*(-?\d+),?\s*\}',profile)}
 assert declared=={(p['signal'],p['pin'],p['af'] if p['af'] is not None else -1) for p in pins},'Rust/JSON pin map drift'
 w=d['iwdg'];assert [w['lsiMinHz'],w['lsiMaxHz']]==e['lsiHz']
 return {'pinAssignments':len(pins),'fastExtiInputs':len(exti),'dmaChannels':len(d['dma']),'channelMappings':len(mapping),'iwdgMinUs':1e6*w['prescaler']*(w['reload']+1)/w['lsiMaxHz'],'iwdgMaxUs':1e6*w['prescaler']*(w['reload']+1)/w['lsiMinHz']}
def main():
 d=json.loads((ROOT/'pin-map.json').read_text());e=json.loads((ROOT/'af-evidence.json').read_text());s=json.loads((ROOT.parent/'electronics/schematic.json').read_text());report=validate(d,e,s);negative=0
 for change in ['pin','af','exti','dma','terminal','device']:
  bad=copy.deepcopy(d)
  if change=='pin':bad['pins'][1]['pin']=bad['pins'][0]['pin']
  elif change=='af':bad['pins'][0]['af']=0
  elif change=='exti':next(p for p in bad['pins'] if p['function'].startswith('EXTI'))['function']='EXTI15'
  elif change=='dma':bad['dma'][1]['channel']=bad['dma'][0]['channel']
  elif change=='terminal':bad['channels'][0]['terminal']='missing'
  else:bad['channels'][0]['devicePin']=bad['channels'][1]['devicePin']
  try:validate(bad,e,s)
  except AssertionError:negative+=1
  else:raise AssertionError('bad '+change+' accepted')
 report.update(status='PASS',negativeFixtures=negative)
 (ROOT/'pin-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
