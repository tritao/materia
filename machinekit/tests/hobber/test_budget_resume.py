#!/usr/bin/env python3
"""Fast launcher regression: fake libraries/results, no physical cuts or native builds.
Run from the repository root; the scheduler retains its normal memory guard.
"""
import tempfile,json,hashlib,subprocess
from pathlib import Path
script=Path('machinekit/examples/flexspline-hobber/run-reference-budget.py').resolve()
settings=dict(moduleMm=.3,teeth=200,faceWidthMm=8,threads=4,workers=4,
 sweepToleranceMm=.000025,pathToleranceMm=.0000015625,radialStepMm=.0015,axialStepMm=.04,windows=92151,
 sourceIds=['mandrel','table-radial','table-axial','encoder-eccentricity','encoder-graduation',
 'hob-radial','hob-axial','tool-radial','tool-axial','hob-profile','sync','outboard-radial','wall'],scope='synthetic launcher fixture')
with tempfile.TemporaryDirectory(prefix='hb11-upgrade-') as temp:
 p=Path(temp);out=p/'out';old=p/'old';face=p/'face';native=p/'native';new=p/'new'
 for d in [out,old,face,native,new]:d.mkdir()
 def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
 for d in [native,new]:(d/'libstockkit_core.so').write_bytes(str(d).encode())
 (old/'provenance.txt').write_text(sha(native/'libstockkit_core.so'))
 for d in [out,old]:
  (d/'settings.json').write_text(json.dumps(settings));(d/'inputs.json').write_text('{}')
 (out/'main.hl').write_bytes(b'not executable: no new physical cuts may launch')
 keys=['fp','Fp','Fr','Falpha','ffalpha','fHalpha','Fbeta','ffbeta','fHbeta']
 raw=dict(values={k:0. for k in keys},grades={k:1 for k in keys},profile=[[0,0]],lead=[[0,0]],pitch=[[0,0]],runoutMm=[0,0],thicknessMm=[.3],warnings=[],extrapolation=['out of scope'])
 for label in ['baseline',*settings['sourceIds'],'combined']:(out/f'{label}.json').write_text(json.dumps(raw))
 base=['python3',str(script),str(out),'--legacy-directory',str(old),'--face-directory',str(face)]
 r=subprocess.run(base+['--native',str(native)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 old_execution=(out/'execution.json').read_bytes()
 new_binary=out/'main-batched.hl';new_binary.write_bytes(b'new frozen bytecode, also never executed by this test')
 proof=dict(method='paired streaming domains',newBinaryPath=str(new_binary),oldBinarySha256=sha(out/'main.hl'),newBinarySha256=sha(new_binary),passed=True,oldNativeSha256=sha(native/'libstockkit_core.so'),newNativeSha256=sha(new/'libstockkit_core.so'),referenceStockHashIdentical=True,pairedExactEndpointsAndProvenance=True,nativeTestsPassed=True,speedup=2)
 (out/'native-upgrade-validation.json').write_text(json.dumps(proof))
 (out/'combined.json').unlink()
 code="import time,json,sys;from pathlib import Path;time.sleep(.5);p=Path(sys.argv[4]);(p/'combined.json').write_text("+repr(json.dumps(raw))+");(p/'combined.log').write_text('HOBBING_ERROR_RESULT '+json.dumps(dict(wallSeconds=.5,threads=4))+'\\n')"
 child=subprocess.Popen(['python3','-c',code,str(out/'main.hl'),'reference-budget',str(out/'inputs.json'),str(out),'combined'])
 (out/'adopt-running.json').write_text(json.dumps({'combined':dict(pid=child.pid,nativeSha256=proof['oldNativeSha256'])}))
 r=subprocess.run(base+['--native',str(new)],capture_output=True,text=True);child.wait();assert r.returncode==0,r.stderr
 record=json.loads((out/'combined-execution.json').read_text());assert record['adopted'] and record['exitCode'] is None and not record['sharedGeometry']
 assert json.loads((out/'execution.json').read_text())['sharedGeometry']
 assert list(out.glob('execution-before-*.json'))[0].read_bytes()==old_execution
 r=subprocess.run(base+['--native',str(new)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 proof['speedup']=.9;(out/'native-upgrade-validation.json').write_text(json.dumps(proof))
 r=subprocess.run(base+['--native',str(new)],capture_output=True,text=True);assert r.returncode!=0 and 'lacks exact parity' in r.stderr
 assert (out/'all-exit-status.txt').read_text()=='0\n'
 print('Upgrade scheduler: exact-proof gate, completed-case preservation, running-child adoption, old execution archive and restart passed; slow candidate rejected')
