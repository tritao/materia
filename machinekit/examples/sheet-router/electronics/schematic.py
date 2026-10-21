#!/usr/bin/env python3
"""SR17 auditable schematic graph. Generates data; never controls hardware."""
import csv, json, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
DRIVES=['X','YLeft','YRight','Z','VFD']
ESTOPS=['cabinet','pendant','rack','tripLeft','tripRight']
def design(model):
    channels=model['beds'][0]['channels']
    assert all(x['channels']==channels for x in model['beds']), 'bed I/O differs'
    # Added carrier/drive channels are explicit requirements, separately identified from CAD I/O.
    extra=[]
    for axis in ['X','YLeft','YRight','Z','Spare']:
        for name,kind in [('step','step'),('dir','direction'),('encoderA','encoder'),('encoderB','encoder'),('enable','output'),('reset','output'),('alarm','input'),('ready','input')]:
            extra.append({'id':f'drive{axis}.{name}','kind':kind})
    for name,kind in [('air.pressure_ok','input'),('spindle.stopped','input'),('probe.trigger','fast-input'),('vacuum.pressure','input'),('safety.reset','input'),('safety.edm','input'),('extractor.run','output'),('extractor.overload','input'),('zBrake.release','output')]:extra.append({'id':name,'kind':kind})
    rows=[];counts={}
    # These are carrier connector pins. MCU assignments are refined/checked in SR15.
    for i,ch in enumerate(channels+extra):
        kind=ch['kind'];index=counts.get(kind,0);counts[kind]=index+1
        board=f'J-{kind.upper()}:{index+1}'
        rows.append(dict(ch,terminal=f'XIO:{i+1}',boardPin=board,origin='assembly' if i<len(channels) else 'SR17 extension',safeValue=0))
    edges=[]
    def edge(a,b,kind='wire'):edges.append({'from':a,'to':b,'kind':kind})
    for lane in ['A','B']:
        last=f'24V.safety{lane}'
        for stop in ESTOPS:
            contact=f'{stop}.NC{lane}';edge(last,contact);last=contact
        relay=f'SR.input{lane}';edge(last,relay);edge(relay,f'SR.delay{lane}','delayed-safety-output')
        for drive in DRIVES:edge(f'SR.delay{lane}',f'{drive}.STO{lane}')
        edge(relay,f'SS1.request{lane}','immediate-safety-output')
        for drive in DRIVES:edge(f'SS1.request{lane}',f'{drive}.controlledStop{lane}')
        edge(f'SR.delay{lane}',f'K{1 if lane=="A" else 2}.coil')
    for a,b in zip(['24V.valves','Fvalve','SR.instant','VFD.zeroSpeed','spindle.stoppedContact','CPU.release','releaseValve'],['Fvalve','SR.instant','VFD.zeroSpeed','spindle.stoppedContact','CPU.release','releaseValve','0V']):edge(a,b)
    power=[{'id':'Q0','kind':'lockable isolator','poles':3,'ratedA':40}, {'id':'QF0','kind':'incoming breaker','ratedA':25}, {'id':'K1','kind':'monitored contactor','ratedA':40}, {'id':'K2','kind':'monitored contactor','ratedA':40}, {'id':'PS1','kind':'24 V isolated PSU','ratedA':10}, {'id':'Fvalve','kind':'24 V fuse','ratedA':2}]
    branches=[{'id':'spindle','mechanicalW':6000,'efficiency':.90,'powerFactor':.90,'breakerA':16}, {'id':'blower','mechanicalW':5500,'efficiency':.88,'powerFactor':.85,'breakerA':16}, {'id':'servos','mechanicalW':3000,'efficiency':.90,'powerFactor':.90,'breakerA':10}, {'id':'extractor','mechanicalW':2200,'efficiency':.85,'powerFactor':.85,'breakerA':10}]
    return {'demandPolicy':{'spindleProcessCapW':1200,'controlsCurrentA':0.5,'connectedRatingExceeds25A':True,'unrestrictedOperationAdmitted':False,'qualification':'SR18 measured phase currents and SR0 supply facts'},'componentReliability':{'assumed':True,'MTTFdYears':30,'diagnosticCoverage':0.90,'CCFPoints':65,'status':'placeholders only; no achieved PL claimed'},'schema':1,'assumed':True,'supply':{'voltageLL':400,'phases':3,'currentA':25,'status':'awaiting SR0 workshop facts','neutralRequired':False,'earthing':'awaiting SR0; PE mandatory; no invented N-PE bond'},'channels':rows,'model':model,'safetyEdges':edges,'drives':DRIVES,'stops':ESTOPS,'powerComponents':power,'powerBranches':branches,'powerNets':[['L1/L2/L3','Q0','QF0','K1','K2','drive branch breakers','drives/VFD/blower/extractor'],['QF0','PS1','fused 24V controls/safety'],['PE','cabinet/star bar','frame/gantry/motor/VFD/chassis/shields']], 'safety':{'targetPL':'d','achievedPL':None,'architecture':'assumed category 3; requires certified components/calculation and fault tests','delaySeconds':3,'brakeApplySeconds':2.9,'manualMonitoredReset':True,'edm':['K1.mirrorNC','K2.mirrorNC'],'relayFault':'immediate STO and contactor drop; category 0','startup':'outputs disabled, manual reset, no automatic restart','release':'series hardware zero-speed AND independent spindle-stopped AND instantaneous safety contact; CPU cannot bypass'},'functions':[{'id':'SF1','function':'e-stop/trip controlled stop then dual STO','targetPL':'d'},{'id':'SF2','function':'prevent release with rotating spindle','targetPL':'d'},{'id':'SF3','function':'prevent unexpected restart using reset/EDM','targetPL':'d'},{'id':'SF4','function':'retain Z under gravity with brake','targetPL':'d'}]}
def reachable(edges,start,end,removed=()):
    seen=set(removed);pending=[start]
    while pending:
        node=pending.pop()
        if node in seen:continue
        if node==end:return True
        seen.add(node);pending.extend(e['to'] for e in edges if e['from']==node)
    return False
def validate(d):
    rows=d['channels'];actual={x['id']:x['kind'] for x in d['model']['beds'][0]['channels']}
    assert len(actual)==len(d['model']['beds'][0]['channels']), 'duplicate model channel'
    for key in ['id','terminal','boardPin']:assert len({x[key] for x in rows})==len(rows),f'duplicate {key}'
    assert {x['id']:x['kind'] for x in rows if x['origin']=='assembly'}==actual,'model-terminal reverse mapping'
    assert all(x['safeValue']==0 for x in rows)
    edges=d['safetyEdges'];checks=4
    for lane in ['A','B']:
        for drive in d['drives']:
            target=f'{drive}.STO{lane}'
            assert reachable(edges,f'24V.safety{lane}',target),f'missing {target}'
            assert not reachable(edges,'24V.safety'+('B' if lane=='A' else 'A'),target), 'cross-lane STO bypass'
            assert not reachable(edges,'24V.valves',target), 'valve feed bypass to STO'
            assert not reachable(edges,f'24V.safety{lane}',target,[f'SR.delay{lane}']), 'delay bypass'
            for stop in d['stops']:
                assert not reachable(edges,f'24V.safety{lane}',target,[f'{stop}.NC{lane}']),f'{stop} bypass to {target}'
                checks+=1
    for gate in ['SR.instant','VFD.zeroSpeed','spindle.stoppedContact','CPU.release']:
        assert reachable(edges,'24V.valves','releaseValve')
        assert all(not reachable(edges,source,'releaseValve',[gate]) for source in ['24V.valves','24V.safetyA','24V.safetyB']),f'release bypasses {gate}'
        checks+=1
    assert d['safety']['manualMonitoredReset'] and len(d['safety']['edm'])==2
    assert d['safety']['brakeApplySeconds']<d['safety']['delaySeconds']
    return checks+22
if __name__=='__main__':
    model=json.loads(Path(sys.argv[1]).read_text());d=design(model);count=validate(d)
    (ROOT/'schematic.json').write_text(json.dumps(d,indent=2)+'\n')
    with (ROOT/'wire-list.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['id','kind','terminal','boardPin','origin','safeValue']);w.writeheader();w.writerows(d['channels'])
    with (ROOT/'safety-wires.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['wire','from','to','kind','areaMm2']);w.writeheader()
        for i,e in enumerate(d['safetyEdges']):w.writerow(dict(e,wire=f'SF-{i+1}',areaMm2=0.75))
    with (ROOT/'power-wires.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['wire','from','to','conductor','areaMm2']);w.writeheader();idx=0
        for phase in ['L1','L2','L3']:
            route=['supply','Q0','QF0','K1','K2','distribution']
            for a,b in zip(route,route[1:]):
                idx+=1;w.writerow(dict(wire=f'P-{idx}',**{'from':a,'to':b},conductor=phase,areaMm2=4))
            for drive in DRIVES+['blower','extractor']:
                for a,b in [('distribution',drive+'.breaker'),(drive+'.breaker',drive)]:
                    idx+=1;w.writerow(dict(wire=f'P-{idx}',**{'from':a,'to':b},conductor=phase,areaMm2=2.5))
        for phase in ['L1','L2']:
            for a,b in [('QF0','QFctrl'),('QFctrl','PS1')]:
                idx+=1;w.writerow(dict(wire=f'P-{idx}',**{'from':a,'to':b},conductor=phase,areaMm2=1.5))
        for body in ['cabinet','frame','gantry']+DRIVES+['blower','extractor']:
            idx+=1;w.writerow(dict(wire=f'P-{idx}',**{'from':'PE.bar','to':body},conductor='PE',areaMm2=4))
    print(json.dumps({'status':'PASS' ,'graphChecks':count,'channels':len(d['channels']),'assemblyChannels':len(model['beds'][0]['channels'])}))
