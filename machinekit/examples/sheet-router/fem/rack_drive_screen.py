"""Single-pose Y-drive bounds; no supplier stiffness or geometry release implied."""
import sys,json,hashlib,gzip,shutil
from pathlib import Path
import brace_macro as bm
from rack_joint import install


def install_option(option):
    previous=bm.variant
    def variant(packet,native,stitches,name):
        p,n,s,changes=previous(packet,native,stitches,name)
        y=next(a for a in p['axes'] if a['axis']=='y');c=y['compliance'];notes=[]
        if option=='gearbox2':c['reducerMPerN']/=2;notes+=['Reducer torsional stiffness x2; mass held fixed as an optimistic bound, no replacement selected.']
        elif option=='mount2':c['hingeMPerN']/=2;notes+=['Preload hinge/motor-mount tangential stiffness x2; mass held fixed. Preload/separation-force and travel checks still required.']
        elif option in ('seat4','seat4both','balanced'):
            for row in p['rackSeatScreen']['axes']:
                if row['axis']=='y' or option in ('seat4both','balanced'):row['seatNPerM']*=4
            notes+=['Mount/bolt stiffness x4 ('+('Y only' if option=='seat4' else 'X and Y')+'); mass held fixed as a best-case stiffness-only bound. RM2 physical design required.']
            if option=='balanced':
                c['hingeMPerN']/=2;c['reducerMPerN']/=2
                notes+=['Combined with 2x Y hinge/mount and reducer stiffness; no actual catalogue replacements or mass additions selected.']
        elif option=='module2':
            c['meshMPerN']*=1.5*20/(2*24)
            for row in p['parts']:
                if row['id'] not in ('rackYLeft','rackYRight'):continue
                # Full section mass/m from reviewed ATLANTA 29-20-105; short
                # offcut drilling and pinion mass/geometry remain unqualified.
                rack_length=3350 if p['workY']==2600 else 3900
                old=row['massKg'];new=4.1*rack_length/1000;factor=new/old
                row['massKg']=new
                for key in row['centroidalInertiaKgMm2World']:row['centroidalInertiaKgMm2World'][key]*=factor
                p['totalMassKg']+=new-old;changes['rackSeatAddedFixedKg']+=new-old
            notes+=['Module 2, 15 teeth retains 100 mm/turn. Mesh stiffness scales with module*face width (assumed), reviewed face 24 mm and rack 4.1 kg/m. Pinion mass held fixed, normal datum/section inertia not redesigned. An 8 mm shelf plus 2 mm replication needs the pinion raised at least 1.916 mm; not a buildable geometry proposal.']
        else:raise ValueError(option)
        c['stiffnessNPerM']=1/sum(v for k,v in c.items() if k.endswith('MPerN'))
        changes.update(driveOption=option,optionNotes=notes,optionDriveStiffnessNPerM=c['stiffnessNPerM'])
        return p,n,s,changes
    bm.variant=variant

if __name__=='__main__':
    i=sys.argv.index('--witnesses');w=Path(sys.argv[i+1]);del sys.argv[i:i+2]
    i=sys.argv.index('--option');option=sys.argv[i+1];del sys.argv[i:i+2]
    output=Path(sys.argv[sys.argv.index('--output')+1])
    install(w);install_option(option);bm.main()
    # These are stiffness-only bounds, not final evidence cases. Preserve all
    # decks, solver logs and modal reports; discard reproducible bulk DAT.
    retention=[]
    for raw in output.rglob('frame.dat.gz'):
        if not raw.with_name('summary.json').is_file() or raw.is_symlink():raise ValueError('Unsafe raw-output retention path')
        with raw.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
        retention.append(dict(path=str(raw.relative_to(output)),sha256=digest,bytes=raw.stat().st_size))
        raw.unlink()
    for deck in output.rglob('frame.inp'):
        compressed=deck.with_suffix('.inp.gz')
        if compressed.exists():continue
        with deck.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
        with deck.open('rb') as source,gzip.open(compressed,'wb',compresslevel=1) as destination:shutil.copyfileobj(source,destination)
        with gzip.open(compressed,'rb') as f:
            if hashlib.file_digest(f,'sha256').hexdigest()!=digest:raise ValueError('Compressed deck differs')
        deck.unlink()
    (output/'raw-retention.json').write_text(json.dumps(retention,indent=2)+'\n')
