"""Run each static load case from the undeformed state in its own CCX process.

Raw step times stay at 0–1. Only the derived concatenated DAT relabels case
time. Raw decks, logs and DAT files are retained, hashed and cited.
"""
import json
from pathlib import Path
import re
from ccx_deck import run_solver
from packet import digest


def split_steps(text):
    chunks=re.split(r'(?m)^\*STEP\s*\n',text)
    if len(chunks)<2:raise ValueError('No ordinary static steps to isolate')
    header=chunks[0];result=[]
    for index,step in enumerate(chunks[1:]):
        if not re.search(r'(?m)^\*STATIC\s*$',step):raise ValueError('Expected ordinary static procedure')
        result.append(header+'*STEP\n'+step)
    return result


def run_independent(ccx,output,timeout,threads,resume=False):
    decks=split_steps((output/'frame.inp').read_text());records=[];data=[];logs=[]
    for index,deck in enumerate(decks,1):
        folder=output/'independent-cases'/f'case-{index}'
        folder.mkdir(parents=True,exist_ok=True);inp=folder/'frame.inp';log=folder/'solver.log';dat=folder/'frame.dat'
        same=inp.exists() and inp.read_text()==deck
        completed=same and dat.exists() and log.exists() and 'Job finished' in log.read_text() and '*ERROR' not in log.read_text().upper() and re.search(rf'Using up to\s+{threads}\s+cpu',log.read_text()) is not None
        inp.write_text(deck)
        if not (resume and completed):run_solver(ccx,folder,timeout,threads)
        else:print(f'Reusing isolated static case: {folder}',flush=True)
        records.append({'case':index,'directory':str(folder),'inputSha256':digest(inp),'resultSha256':digest(dat),'logSha256':digest(log)})
        raw=dat.read_text()
        # Reporting labels only: raw single-case solver output is preserved above.
        combined=re.sub(r'(?i)(\band time\s+)([-+0-9.EeDd]+)',lambda m:m.group(1)+f'{index:.7E}',raw)
        data.append(combined);logs.append(f'Independent case {index}: {folder}\n'+log.read_text())
    (output/'frame.dat').write_text('\n'.join(data));(output/'solver.log').write_text('\n'.join(logs))
    (output/'independent-cases.json').write_text(json.dumps(records,indent=2)+'\n')
    return records
