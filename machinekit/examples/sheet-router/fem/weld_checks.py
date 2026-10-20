"""Effective-length and fatigue-basis regression fixtures for the SR6 packet."""
import math,unittest
from weld_effective import effective_stitches
from weld_report import FATIGUE_ALLOWABLE,FATIGUE_CYCLES


class WeldChecks(unittest.TestCase):
    def effective(self,points,leg=5,full=False):
        native=dict(seams=[]);lines=[]
        for i,(a,b) in enumerate(zip(points,points[1:])):
            id=str(i);native['seams'].append(dict(id=id,a=dict(member='a'),b=dict(member='b')))
            lines.append(dict(group='G',seam=id,a=a,b=b,lengthMm=math.dist(a,b),legMm=leg,fullSizeEnds=full))
        accepted,rejected=effective_stitches(native,lines)
        return sum(r['lengthMm'] for r in accepted),rejected

    def test_open_start_end_deductions(self):
        length,rejected=self.effective([[0,0,0],[40,0,0]])
        self.assertAlmostEqual(length,40-2*5/math.sqrt(2));self.assertEqual(rejected,[])

    def test_short_run_has_no_capacity(self):
        length,rejected=self.effective([[0,0,0],[30,0,0]])
        self.assertEqual(length,0);self.assertEqual(len(rejected),1)

    def test_connected_returns_are_one_run(self):
        length,rejected=self.effective([[0,2.5,0],[0,0,0],[80,0,0],[80,2.5,0]])
        self.assertAlmostEqual(length,85-2*5/math.sqrt(2));self.assertEqual(rejected,[])

    def test_closed_run_has_no_open_ends(self):
        length,rejected=self.effective([[0,0,0],[40,0,0],[40,40,0],[0,40,0],[0,0,0]])
        self.assertEqual(length,160);self.assertEqual(rejected,[])

    def test_tabbed_foot_full_size_and_default_reduction(self):
        full,_=self.effective([[0,0,0],[92,0,0]],8,True)
        normal,_=self.effective([[0,0,0],[92,0,0]],8,False)
        self.assertEqual(full,92);self.assertAlmostEqual(normal,92-2*8/math.sqrt(2))

    def test_tabs_do_not_waive_minimum_length(self):
        length,rejected=self.effective([[0,0,0],[30,0,0]],8,True)
        self.assertEqual(length,0);self.assertEqual(len(rejected),1)

    def test_approved_life_and_two_slope_cutoff(self):
        self.assertEqual(FATIGUE_CYCLES,100000000)
        self.assertAlmostEqual(FATIGUE_ALLOWABLE,12.669281670375343)


if __name__=='__main__':unittest.main()
