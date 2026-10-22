"""Prepared-weld counterpart of the review-only rack-seat joint screen."""
import sys
from pathlib import Path
import brace_study as bs
import brace_macro as bm
from rack_joint import install

class JointPrepared(bs.BraceModel):
    def __init__(self,h,packet,*args,**kw):
        self.seatK={r['axis']:r['seatNPerM']/1000 for r in packet['rackSeatScreen']['axes']}
        super().__init__(h,packet,*args,**kw)
    def spring(self,a,b,k,group):
        if group not in ('Xdrive','Ydrive'):return super().spring(a,b,k,group)
        axis='x' if group=='Xdrive' else 'y';i=0 if axis=='x' else 1
        name='rackSeat'+str(len(self.springs));root=self.root(name,self.roots[b]['p'],name)
        super().spring(a,root,k,group)
        seat=[1e12]*6;seat[i]=self.seatK[axis]
        super().spring(root,b,seat,'rackSeatBolts')

if __name__=='__main__':
    index=sys.argv.index('--witnesses');witnesses=Path(sys.argv[index+1]);del sys.argv[index:index+2]
    install(witnesses);bs.disk=bm.disk;bs.variant=bm.variant;bs.BraceModel=JointPrepared;bs.main()
