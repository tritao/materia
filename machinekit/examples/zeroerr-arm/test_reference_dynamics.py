import unittest
import numpy as np
from reference_dynamics import reference_study,central_inertia,terms


class FrozenDynamicsTests(unittest.TestCase):
    def test_six_whole_modules_no_shell_masses(self):
        s=reference_study();modules=[b for b in s.bodies if b[0].startswith('module')]
        self.assertEqual(len(modules),6)
        self.assertFalse(any(b[0].startswith('joint') for b in s.bodies))
        self.assertAlmostEqual(sum(b[1] for b in modules),3*4.49+3*1.24)
        self.assertEqual(modules[1][4],[0,1]);self.assertEqual(modules[2][4],[0,1])

    def test_parallel_axis_units(self):
        c=np.array([.1,0,0]);at_origin=np.diag([.01,.04,.04])
        np.testing.assert_allclose(central_inertia(2,c,at_origin),np.diag([.01,.02,.02]))

    def test_hand_pose_payload_increment(self):
        q=np.array([0,-np.pi/2,0,0,0,0])
        g,_=terms(reference_study(5),q);bare,_=terms(reference_study(0),q)
        self.assertAlmostEqual(g[1]-bare[1],5*9.80665*.912,places=7)

    def test_energy_and_reference_solver(self):
        s=reference_study();q=np.array([.17,-.63,.48,.29,-.37,.23]);g,m=terms(s,q)
        expected=s.terms(q);np.testing.assert_allclose(g,expected[0],atol=1e-9)
        np.testing.assert_allclose(m,expected[1],atol=1e-9)
        grad=[]
        for i in range(6):
            step=np.eye(6)[i]*1e-6
            grad.append((s.terms(q+step)[3]-s.terms(q-step)[3])/2e-6)
        np.testing.assert_allclose(g,grad,atol=1e-6)
        self.assertGreaterEqual(np.linalg.eigvalsh(m).min(),-1e-10)


if __name__=='__main__':unittest.main()
