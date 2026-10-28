"""Independent elastic identities; run under the shared resource guard."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mount_frame import Frame,rectangle,beam_matrix


class LocalFrameTests(unittest.TestCase):
    def test_beam_cantilever_units_shear_and_torsion(self):
        model=Frame();model.node('root',(0,0,0));model.node('tip',(200,0,0));s=rectangle(20,10)
        model.beam('root','tip',s,(0,1,0),'metal');model.fix('root')
        e=210000.;g=e/2.6
        for load,coordinate,expected in [([100,0,0,0,0,0],0,100*200/(e*s.area)),
            ([0,100,0,0,0,0],1,100*200**3/(3*e*s.iw)+100*200/(g*s.shear_v)),
            ([0,0,100,0,0,0],2,100*200**3/(3*e*s.iv)+100*200/(g*s.shear_w)),
            ([0,0,0,1000,0,0],3,1000*200/(g*s.torsion))]:
            result=model.solve({'tip':load})
            self.assertAlmostEqual(result['displacement']['tip'][coordinate]/expected,1.,places=9)
            self.assertAlmostEqual(sum(result['energyShare'].values()),1.,places=12)

    def test_rigid_offset_transfers_moment_and_reciprocity(self):
        model=Frame();model.node('ground',(0,0,0));model.node('load',(0,0,80));model.rigid('ground','load')
        model.spring('ground',None,[10000,20000,30000,400000,500000,600000],'physical-spring')
        a=np.array([1.,0,0,0,0,0]);b=np.array([0.,1,0,0,0,0])
        first=model.solve({'load':a});second=model.solve({'load':b})
        self.assertAlmostEqual(first['work'],1/10000+80**2/500000,places=12)
        self.assertAlmostEqual(float(a@second['displacement']['load']),float(b@first['displacement']['load']),places=12)

    def test_global_rotation_and_rigid_body_nullspace(self):
        s=rectangle(23,11);k=beam_matrix((2,3,4),(202,3,4),s,(0,1,0))
        for v in np.eye(3):
            translation=np.concatenate([v,[0,0,0],v,[0,0,0]])
            self.assertLess(np.linalg.norm(k@translation),1e-8)
        model=Frame();model.node('a',(0,0,0));model.node('b',(0,200,0));model.beam('a','b',s,(0,0,1),'metal');model.fix('a')
        result=model.solve({'b':[0,0,100,0,0,0]})
        expected=100*200**3/(3*210000*s.iw)+100*200/((210000/2.6)*s.shear_v)
        self.assertAlmostEqual(result['displacement']['b'][2]/expected,1.,places=9)

if __name__=='__main__':unittest.main()
