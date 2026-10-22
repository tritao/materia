"""Review-only seat/bolt series springs and CAD mass on frozen brace packets.

Uses brace_macro's serial solver and modal/energy/reciprocity guards. Does not
change frozen CAD, source packets or the existing drive compliance assumptions.
"""
import copy,json,sys,shutil
from pathlib import Path
import brace_macro as bm
import modes
modes.GROUPS["rack seats and bolts"]=["rackSeatBolts"]
from rack_seat_compliance import screen

class JointMacro(bm.Macro):
    def __init__(self,h,packet):
        self.seatK={r['axis']:r['seatNPerM']/1000 for r in packet['rackSeatScreen']['axes']}
        super().__init__(h,packet)
    def spring(self,a,b,k,group):
        if group not in ('Xdrive','Ydrive'):return super().spring(a,b,k,group)
        axis='x' if group=='Xdrive' else 'y';i=0 if axis=='x' else 1
        name='rackSeat'+str(len(self.springs));root=self.root(name,self.roots[b]['p'],name)
        super().spring(a,root,k,group)
        seat=[1e12]*6;seat[i]=self.seatK[axis]
        # Other dummy-node DOFs follow fixed support. They do not constrain
        # the original moving body because original drive k is axial only.
        super().spring(root,b,seat,'rackSeatBolts')


def install(witnesses):
    def disk_reserve():
        free=shutil.disk_usage(Path.cwd()).free
        if free<8_200_000_000:raise RuntimeError('STOP: preserve 8 GB floor plus current-solve reserve; free='+str(free))
    bm.disk=disk_reserve
    original=bm.variant
    def variant(packet,native,stitches,name):
        if name!='baseline':raise ValueError('Braces retained; rack study accepts baseline only')
        p,n,s,changes=original(packet,native,stitches,name)
        bed=int(p['workY']) if 'workY' in p else int(max(q['y'] for q in p['poses']))
        witness=json.loads((witnesses/f'layout-y{bed}.json').read_text())
        axes={r['axis']:r for r in p['axes']}
        p['rackSeatScreen']={'axes':[screen(axis,axes[axis]['compliance']['stiffnessNPerM'],42.509505690810 if axis=='y' else 34.509505690810) for axis in ('x','y')]}
        addedMoving=addedFixed=0.
        for row in witness['parts']:
            r=copy.deepcopy(row);old=next((t for t in p['parts'] if t['id']==r['id']),None)
            delta=r['massKg']-(old['massKg'] if old else 0)
            if old:p['parts'].remove(old)
            p['parts'].append(r);p['totalMassKg']+=delta
            if r['id'].startswith('rackXSeat') or r['id']=='rackX':
                if old:
                    for item in axes['y']['movingMembers']:
                        if item['id']==r['id']:item['kg']=r['massKg']
                else:axes['y']['movingMembers'].append({'id':r['id'],'kg':r['massKg']})
                axes['y']['movingMassKg']+=delta;addedMoving+=delta
            else:addedFixed+=delta
        # Rack drilling and corrected rack pose use CAD mass/inertia. Small
        # frame/rail-pad drilling mass reductions are conservatively omitted.
        changes.update(rackSeatAddedMovingYKg=addedMoving,rackSeatAddedFixedKg=addedFixed,
            qualification='Proposal screen; no J5 control coverage; frame/rail-pad drill mass reduction omitted')
        return p,n,s,changes
    bm.variant=variant;bm.Macro=JointMacro

if __name__=='__main__':
    # Remove our own argument before passing the normal brace_macro CLI.
    index=sys.argv.index('--witnesses');witnesses=Path(sys.argv[index+1]);del sys.argv[index:index+2]
    install(witnesses);bm.main()
