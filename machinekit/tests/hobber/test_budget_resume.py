#!/usr/bin/env python3
"""Fast launcher regression: fake libraries/results, no physical cuts or native builds.
Run from the repository root; the scheduler retains its normal memory guard.
"""
import tempfile,json,hashlib,subprocess,shutil
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
 # A dead adoption with only a partial log must retry, preserving the original
 # evidence. Use a synthetic launcher root so no real HashLink/native cut can run.
 synthetic=p/'synthetic';copied=synthetic/'machinekit/examples/flexspline-hobber/run-reference-budget.py'
 copied.parent.mkdir(parents=True);shutil.copyfile(script,copied)
 fake=synthetic/'haxeon/.tools/hashlink/hl';fake.parent.mkdir(parents=True)
 fake.write_text("#!/usr/bin/env python3\nimport sys,json\nfrom pathlib import Path\np=Path(sys.argv[4]);mode=sys.argv[5]\n(p/(mode+'.json')).write_text("+repr(json.dumps(raw))+ ")\nprint('HOBBING_ERROR_RESULT '+json.dumps(dict(wallSeconds=.01,threads=4)))\n")
 fake.chmod(0o755)
 (out/'combined.json').unlink();partial='HOBBING_ERROR_PROGRESS {"windows":64900,"total":92151}\n'
 (out/'combined.log').write_text(partial)
 # The completed child is gone; its stale adoption remains deliberately.
 preserved=(out/'mandrel.json').read_bytes()
 r=subprocess.run(['python3',str(copied),*base[2:],'--native',str(new)],capture_output=True,text=True)
 assert r.returncode==0,r.stderr
 assert 'RETRY_INTERRUPTED_ADOPTION combined' in r.stdout
 archives=list(out.glob('combined-interrupted-*.log'));assert len(archives)==1 and archives[0].read_text()==partial
 assert (out/'mandrel.json').read_bytes()==preserved
 assert json.loads((out/'combined-resume.json').read_text())['originalAdoption']['pid']==child.pid
 # A completed but invalid inspection must not be retried or overwritten.
 (out/'combined.json').write_text(json.dumps({'values':{}}));(out/'combined.log').write_text('HOBBING_ERROR_RESULT {}\n')
 r=subprocess.run(['python3',str(copied),*base[2:],'--native',str(new)],capture_output=True,text=True)
 assert r.returncode!=0 and len(list(out.glob('combined-interrupted-*.log')))==1
 (out/'combined.json').write_text(json.dumps(raw))
 (out/'all-exit-status.txt').write_text('0\n')
 proof['speedup']=.9;(out/'native-upgrade-validation.json').write_text(json.dumps(proof))
 r=subprocess.run(base+['--native',str(new)],capture_output=True,text=True);assert r.returncode!=0 and 'lacks exact parity' in r.stderr
 assert (out/'all-exit-status.txt').read_text()=='0\n'
 print('Upgrade scheduler: exact-proof gate, completed-case preservation, running-child adoption, old execution archive, interrupted-adoption retry and restart passed; invalid completed inspection and slow candidate rejected')
