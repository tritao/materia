"""Independent affine-strain patch and axial bar checks for local tetrahedra."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mount_solid import tetra_stiffness, SolidFixture
from mount_motor import pin_stiffness, tooth_reaction


class LocalSolidTests(unittest.TestCase):
    def test_tooth_force_is_normal_to_the_actual_loft_generator(self):
        for hand, sign in [('right',1),('left',-1)]:
            row=dict(hand=hand,helixAngle=.34,pressureAngle=.35)
            generator=np.array([sign*np.tan(.34),0,1])
            for force in (-1.,1.):
                self.assertAlmostEqual(float(tooth_reaction(row,force)@generator),0.,places=12)

    def test_simply_supported_pin_force_and_couple(self):
        diameter, span, e, g = 10.,25.,210000.,210000/2.6
        area, inertia = np.pi*diameter**2/4,np.pi*diameter**4/64
        radial,tilt = pin_stiffness(diameter,span)
        self.assertAlmostEqual(1/radial,span**3/(48*e*inertia)+span/(4*.9*g*area),places=12)
        self.assertAlmostEqual(1/tilt,span/(12*e*inertia)+1/(.9*g*area*span),places=12)

    def test_affine_strain_energy_and_rigid_motion(self):
        points = np.array([[0., 0, 0], [2., 0, 0], [0., 3., 0], [0., 0., 4.]])
        k, volume = tetra_stiffness(points, np.array([[0, 1, 2, 3]]), youngs=200000., poisson=0.)
        self.assertAlmostEqual(volume, 4.)
        u = np.zeros((4, 3)); u[:, 0] = points[:, 0]*.001
        self.assertAlmostEqual(float(u.ravel()@k@u.ravel()/2), .5*200000*.001**2*volume)
        for direction in np.eye(3):
            self.assertLess(np.linalg.norm(k@np.tile(direction, 4)), 1e-8)
        rigid_rotation = np.cross(np.array([.01, -.02, .03]), points)
        self.assertLess(np.linalg.norm(k@rigid_rotation.ravel()), 1e-8)

    def test_bar_axial_extension_with_finite_root(self):
        xyz = np.array([[x, y, z] for z in (0., 10.) for y in (0., 10.) for x in (0., 100.)])
        tets = np.array([[0,1,3,7], [0,3,2,7], [0,2,6,7], [0,6,4,7], [0,4,5,7], [0,5,1,7]])
        model = SolidFixture(); model.port('root', (0,5,5)); model.port('tip', (100,5,5))
        model.solid('bar', xyz, tets, {'root':lambda p:p[:,0] == 0, 'tip':lambda p:p[:,0] == 100},
                    youngs=200000., poisson=0.)
        model.spring('root', None, [1e12]*6, 'finite root')
        result = model.solve({'tip':[100.,0,0,0,0,0]})
        extension = result['displacement']['tip'][0]-result['displacement']['root'][0]
        self.assertAlmostEqual(extension/(100*100/(200000*100)), 1., places=7)
        self.assertLess(result['residualRelative'], 1e-7)


if __name__ == '__main__': unittest.main()
