import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('bench',Path(__file__).with_name('evaluate-j2-bench.py'))
bench=importlib.util.module_from_spec(spec);spec.loader.exec_module(bench)

class BenchTests(unittest.TestCase):
    def test_measured_mass_lever_units_and_sign(self):
        self.assertAlmostEqual(bench.measured_torque([dict(name='load',massKg=10,signedHorizontalCogMm=500)]),49.03325)
        self.assertAlmostEqual(bench.measured_torque([dict(name='load',massKg=10,signedHorizontalCogMm=-500)]),-49.03325)
    def test_missing_measurements_are_not_zero(self):
        with self.assertRaises(ValueError):bench.measured_torque([dict(name='unweighed',massKg=None,signedHorizontalCogMm=500)])
    def test_fixture_overload_rejected(self):
        data=dict(measuredBodies=[dict(name='load',massKg=10,signedHorizontalCogMm=500)],fixture=dict(approvedCapacityNm=20,reviewed=True,outputMountedUpstream=True,movingHousing=True))
        with self.assertRaises(ValueError):bench.evaluate(data)

if __name__=='__main__':unittest.main()
