"""Preempt only this study's large CCX processes before host low-memory pressure.

The shared heavy-run limits remain in force. Retry occurs only after this guard
sent SIGTERM; a numerical/solver failure is returned immediately.
"""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def available_bytes():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):return int(line.split()[1])*1024
    raise RuntimeError('Linux MemAvailable is required by the resource guard')


def owned_solvers(ccx,roots,proc=Path('/proc')):
    ccx=ccx.resolve();roots=[p.resolve() for p in roots];result=[]
    for entry in proc.iterdir():
        if not entry.name.isdigit():continue
        try:
            if (entry/'exe').resolve(strict=True)!=ccx:continue
            cwd=(entry/'cwd').resolve(strict=True)
            if not any(cwd.is_relative_to(root) for root in roots):continue
            if 'heavy-mill-' not in (entry/'cgroup').read_text():continue
            rss=next((int(line.split()[1])*1024 for line in (entry/'status').read_text().splitlines() if line.startswith('VmRSS:')),0)
            if rss<512*1024**2:continue
            result.append({'pid':int(entry.name),'cwd':cwd,'rssBytes':rss})
        except (OSError,ValueError,StopIteration):continue
    return result


def run(command,ccx,roots,minimum_available=4.5*1024**3):
    attempt=0
    while True:
        attempt+=1;stopped=False;stopped_pids=set();stopped_paths=set()
        process=subprocess.Popen(command,start_new_session=True)
        try:
            while process.poll() is None:
                free=available_bytes()
                if free<minimum_available:
                    for item in owned_solvers(ccx,roots):
                        if item['pid'] in stopped_pids:continue
                        try:os.kill(item['pid'],signal.SIGTERM)
                        except ProcessLookupError:continue
                        stopped=True;stopped_pids.add(item['pid']);stopped_paths.add(item['cwd'])
                        record={**item,'cwd':str(item['cwd']),'availableBytes':free,'thresholdBytes':minimum_available,
                            'utc':datetime.now(timezone.utc).isoformat(),'attempt':attempt,'reason':'This guard preempted its owned CCX process before host earlyoom threshold'}
                        (item['cwd']/'resource-stop.json').write_text(json.dumps(record,indent=2)+'\n')
                        print('Resource guard stopped owned CCX '+str(item['pid'])+'; completed reports remain available for exact-deck resume.',flush=True)
                time.sleep(.25)
            code=process.wait()
        except BaseException:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            for item in owned_solvers(ccx,roots):
                try:os.kill(item['pid'],signal.SIGTERM)
                except ProcessLookupError:pass
            raise
        if code==0 or not stopped:return code
        if any((path/'solver.log').exists() and '*ERROR' in (path/'solver.log').read_text().upper() for path in stopped_paths):return code
        print('Retrying this study after resource-pressure interruption; numerical errors are never retried automatically.',flush=True)
        time.sleep(30)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ccx',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,action='append',required=True)
    parser.add_argument('--minimum-available-gib',type=float,default=4.5)
    parser.add_argument('command',nargs=argparse.REMAINDER);args=parser.parse_args()
    command=args.command[1:] if args.command and args.command[0]=='--' else args.command
    if not command or not 3.5<=args.minimum_available_gib<=16:parser.error('Command and a 3.5–16 GiB live reserve are required')
    def stop(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    raise SystemExit(run(command,args.ccx,args.output_root,args.minimum_available_gib*1024**3))
