#!/usr/bin/env python3
import copy,json
from pathlib import Path
from schematic import validate
root=Path(__file__).resolve().parent
d=json.loads((root/'schematic.json').read_text());n=validate(d);negative=0
for a,b in [('24V.valves','releaseValve'),('24V.valves','CPU.release'),('24V.safetyA','X.STOA'),('24V.safetyB','Z.STOB'),('24V.safetyB','X.STOA'),('24V.safetyA','releaseValve')]:
 bad=copy.deepcopy(d);bad['safetyEdges'].append({'from':a,'to':b,'kind':'fault injected'})
 try:validate(bad)
 except AssertionError:negative+=1
 else:raise AssertionError('bypass not detected: '+a+' -> '+b)
for field in ['terminal','boardPin','id']:
 bad=copy.deepcopy(d);bad['channels'][1][field]=bad['channels'][0][field]
 try:validate(bad)
 except AssertionError:negative+=1
 else:raise AssertionError('duplicate '+field+' not detected')
bad=copy.deepcopy(d);bad['channels'].pop(0)
try:validate(bad)
except AssertionError:negative+=1
else:raise AssertionError('unwired model channel not detected')
report={'status':'PASS','graphChecks':n,'negativeFixtures':negative,'channels':len(d['channels']),'assemblyChannels':sum(x['origin']=='assembly' for x in d['channels']),'achievedPL':None}
(root/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
