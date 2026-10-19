"""Run nine-pose upper-bound modes first, then machine-loop static cases.

Each study enters the shared heavy-job queue separately. CAD packets must be
exported together with --pose-matrix true; this command never edits CAD.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from packet import digest,validate


def reusable(path,manifest,config):
    if not path.exists():return False
    report=json.loads(path.read_text())
    if not report.get('converged') or report['manifestSha256']!=digest(manifest) or report['configurationSha256']!=digest(config):return False
    if 'budgetConfigurationSha256' in report and report['budgetConfigurationSha256']!=digest(Path(__file__).with_name('compliance-budget.json')):return False
    return all(digest(Path(__file__).with_name(name))==value for name,value in report['sourceSha256'].items())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packets',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ccx',required=True);parser.add_argument('--heavy-run',type=Path,default=Path('/home/joao/dev/materia-cache/tools/heavy-run'))
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('frame-study.json'))
    parser.add_argument('--sizes',nargs='+',type=float,default=[40,30]);parser.add_argument('--resume',action='store_true')
    parser.add_argument('--threads',type=int,choices=[1,2,4],default=1)
    parser.add_argument('--stage',choices=['all','modal','static'],default='all')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    entries=json.loads((args.packets/'poses.json').read_text())
    if len(entries)!=9 or entries[0]['id']!='center':raise ValueError('Expected center plus eight corners')
    records=[];fixed=None
    for entry in entries:
        manifest=args.packets/entry['id']/'manifest.json';validate(manifest)
        packet=json.loads(manifest.read_text())
        if packet['jointsMm']!={k:entry[k] for k in ('x','y','z')}:raise ValueError('Pose index/packet mismatch')
        hashes={p['id']:p['stepSha256'] for p in packet['parts'] if p['id'].startswith('frame/') and p['stepFile']}
        if fixed is None:fixed=hashes
        if hashes!=fixed:raise ValueError('Fixed-frame geometry changed across pose packets')
        records.append((entry,manifest))
    def study(script,manifest,output,extra):
        if args.resume and reusable(output/'report.json',manifest,args.config):return
        command=[str(args.heavy_run),'--slots','4','--mem','6G','--need','4G','--margin','6G','--label','mill-pose-study','--',sys.executable,
            str(Path(__file__).with_name(script)),str(manifest),'--ccx',args.ccx,'--config',str(args.config),'--output',str(output),
            '--threads',str(args.threads),'--sizes',*[str(v) for v in args.sizes],*extra]
        print('Starting '+str(output),flush=True)
        subprocess.run(command,check=True)
    if args.stage in ('all','modal'):
        for entry,manifest in records:study('modal_study.py',manifest,args.output/entry['id']/'modal',['--resume'])
    if args.stage in ('all','static'):
        for entry,manifest in records:
            for case in ('spindle-nose','test-bar-20'):
                study('static_study.py',manifest,args.output/entry['id']/case,['--tool-case',case,'--resume'])
    return 0

if __name__=='__main__':raise SystemExit(main())
