import math
from pathlib import Path
import tempfile
import unittest
from modal_motion import frd_displacements,relative_motion

class ModalMotionChecks(unittest.TestCase):
    def test_rigid_reference_observation_and_common_motion_cancellation(self):
        model={'references':{'head':{'translation':1,'rotation':2,'pointMm':[0,0,0]},
            'table':{'translation':3,'rotation':4,'pointMm':[10,0,0]}}}
        u={1:[1,2,3],2:[0,0,.1],3:[0,0,0],4:[0,0,0]}
        self.assertEqual(relative_motion(model,u,[10,20,30]),[-1,3,3])
        u[3]=[1,3,3];u[4]=[0,0,.1]
        self.assertEqual(relative_motion(model,u,[10,20,30]),[0,0,0])

    def test_native_fixed_width_negative_components(self):
        # Literal CCX 2.21 field layout: adjacent negative numbers have no spaces.
        text='''    1PSTEP                         1           1           1
    1PGM                1.000000E+00
    1PMODE                         1
  100CL  101 48.64285475       98352                     2    1MODAL      1
 -4  DISP        4    1
 -1     98295 3.29228E-01-1.02469E+00-2.60363E-02
 -1     98296 1.00000E+00 2.00000E+00 3.00000E+00
 -3
'''
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'frame.frd';path.write_text(text)
            fields=frd_displacements(path,{98295})
        self.assertEqual(len(fields),1)
        self.assertEqual(fields[0]['mode'],1)
        self.assertEqual(fields[0]['generalizedMass'],1)
        self.assertEqual(fields[0]['u'],{98295:[.329228,-1.02469,-.0260363]})

    def test_analytical_platform_modal_kinetic_fractions(self):
        path=Path(__file__).resolve().parents[1]/'build/fem/modal-fixture-4threads/frame.frd'
        if not path.exists():self.skipTest('Generate the analytical four-thread modal fixture first')
        fields=frd_displacements(path,set(range(1,5))|set(range(11,17)))
        self.assertEqual(len(fields),6)
        expected=[12/12.4]*3+[600/640,800/960,1000/1200]
        for field,fraction in zip(fields,expected):
            lump=sum(.002*sum(v*v for v in field['u'][n]) for n in range(11,17))
            patch=sum(.0001*sum(v*v for v in field['u'][n]) for n in range(1,5))
            self.assertAlmostEqual((lump+patch)/field['generalizedMass'],1,delta=1e-5)
            self.assertAlmostEqual(lump/field['generalizedMass'],fraction,delta=1e-5)

if __name__=='__main__':unittest.main()
