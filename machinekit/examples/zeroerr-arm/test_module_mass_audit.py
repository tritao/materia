"""Analytical checks for sampled torque/load bounds and original mass bookkeeping."""
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit',ROOT/'module-mass-audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class AuditTests(unittest.TestCase):
    def test_static_pendulum_capacity(self):
        result=audit.capacity(np.array([[2.]]),np.zeros((1,1,1)),np.array([[3.]]),
                              np.zeros((1,1,1)),np.array([8.]),0,1,5)
        self.assertAlmostEqual(result['totalFlangeLoadKg'],2,places=10)

    def test_acceleration_and_factor_capacity(self):
        # 1.2 * (2 + 3L + (1 + 2L)*1 rad/s^2) <= 12 gives L <= 1.4.
        result=audit.capacity(np.array([[2.]]),np.array([[[1.]]]),np.array([[3.]]),
                              np.array([[[2.]]]),np.array([12.]),180/np.pi,1.2,5)
        self.assertAlmostEqual(result['totalFlangeLoadKg'],1.4,places=10)

    def test_empty_failure_is_not_a_zero_payload_rating(self):
        result=audit.capacity(np.array([[9.]]),np.zeros((1,1,1)),np.array([[-3.]]),
                              np.zeros((1,1,1)),np.array([8.]),0,1,5)
        self.assertIsNone(result['totalFlangeLoadKg'])
        self.assertEqual(result['status'],'arm_alone_exceeds_assumed_rating')

    def test_original_load_decomposition_and_moving_side(self):
        engineering=audit.module('audit_fixture_engineering',ROOT/'engineering-study.py')
        geometry=json.loads((ROOT/'build/engineering-input.json').read_text())
        config=json.loads((ROOT/'original-comparison-config.json').read_text())
        original=engineering.Study(geometry,config)
        # Original J2 housing moves; its output is upstream. J3 reverses that arrangement.
        for name,_,_,_,active in original.bodies:
            if name=='joint2-stator': self.assertIn(1,active)
            if name=='joint2-rotor': self.assertNotIn(1,active)
            if name=='joint3-stator': self.assertNotIn(2,active)
            if name=='joint3-rotor': self.assertIn(2,active)
        q=np.array([.17,-.63,.48,.29,-.37,.23])
        config['application'].update(toolKg=0,payloadKg=0)
        bare=engineering.Study(geometry,config)
        config['application']['payloadKg']=1
        one=engineering.Study(geometry,config)
        g,m,_,_=bare.terms(q);p,n,_,_=one.terms(q)
        actual=original.terms(q)
        np.testing.assert_allclose(g+5*(p-g),actual[0],atol=1e-10)
        np.testing.assert_allclose(m+5*(n-m),actual[1],atol=1e-10)


if __name__=='__main__': unittest.main()
