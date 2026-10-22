"""Independent small-load checks for the brace study's vector fatigue envelope."""
import itertools,math,unittest
import numpy as np
from brace_motion import traction_envelope


class VectorEnvelopeChecks(unittest.TestCase):
    def calculate(self,history,cut=None,weight=None,tail=None):
        history=np.asarray(history,dtype=float)
        return traction_envelope(history,np.zeros((len(history),3,4)) if cut is None else cut,
            np.zeros((len(history),3)) if weight is None else weight,
            np.zeros((len(history),3)) if tail is None else tail)

    def test_constant_vector_full_reversal(self):
        r=self.calculate([[[3,0],[4,0],[0,0]]])
        self.assertAlmostEqual(r['fatigueHistoryRangeMPa'],10)
        self.assertAlmostEqual(r['staticDesignMPa'],7.5)

    def test_component_peaks_are_not_simultaneous(self):
        r=self.calculate([[[3,0],[0,4],[0,0]]])
        self.assertAlmostEqual(r['fatigueRangeMPa'],8)
        self.assertAlmostEqual(r['fatigueComponentIntervalBoundMPa'],10)

    def test_cutting_sign_allowance(self):
        cut=np.zeros((1,3,4));cut[0,0,0]=1
        r=self.calculate([[[3,0],[4,0],[0,0]]],cut=cut)
        self.assertAlmostEqual(r['fatigueRangeMPa'],2*math.sqrt(32))

    def test_gravity_does_not_reverse(self):
        r=self.calculate([[[0],[0],[0]]],weight=np.array([[0.,0.,10.]]))
        self.assertEqual(r['fatigueRangeMPa'],0)
        self.assertAlmostEqual(r['staticDesignMPa'],13.5)

    def test_future_tail_can_govern(self):
        r=self.calculate([[[3,0],[4,0],[0,0]]],tail=np.array([[0.,0.,6.]]))
        self.assertAlmostEqual(r['fatigueHistoryRangeMPa'],10)
        self.assertAlmostEqual(r['fatigueTailRangeBoundMPa'],12)
        self.assertAlmostEqual(r['fatigueRangeMPa'],12)

    def test_exhaustive_signed_history(self):
        rng=np.random.default_rng(663);history=rng.normal(size=(3,3,11));cut=rng.normal(size=(3,3,4));weight=rng.normal(size=(3,3))
        r=self.calculate(history,cut,weight);fatigue=static=0
        for point in range(3):
            for time in range(11):
                for signs in itertools.product((-1.,1.),repeat=4):
                    for direction in (-1,1):
                        variable=direction*history[point,:,time]+cut[point]@signs
                        fatigue=max(fatigue,2*np.linalg.norm(variable))
                        static=max(static,np.linalg.norm(1.35*weight[point]+1.5*variable))
        self.assertAlmostEqual(r['fatigueRangeMPa'],fatigue)
        self.assertAlmostEqual(r['staticDesignMPa'],static)
        self.assertLessEqual(fatigue,r['fatigueComponentIntervalBoundMPa'])

    def test_zero(self):
        r=self.calculate([[[0],[0],[0]]])
        self.assertEqual(r['fatigueRangeMPa'],0)
        self.assertEqual(r['staticDesignMPa'],0)
        self.assertTrue(r['fatiguePass'] and r['staticPass'])


if __name__=='__main__':unittest.main()
