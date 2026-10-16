"""Independent mechanics checks for the arm option comparison."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('options',ROOT/'arm-options-study.py')
options=importlib.util.module_from_spec(spec);spec.loader.exec_module(options)
engineering=options.load_module('option_fixture_engineering',ROOT/'engineering-study.py')


class OptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry=json.loads((ROOT/'build/engineering-input.json').read_text())
        cls.original=json.loads((ROOT/'original-comparison-config.json').read_text())
        cls.config=json.loads((ROOT/'arm-options-config.json').read_text())
        cls.materials=json.loads((ROOT/'build/module-audit/audit.json').read_text())['materialAudit']

    def test_vectorized_solver_matches_reference_and_energy_gradient(self):
        study=engineering.Study(self.geometry,self.original)
        for q in [np.zeros(6),np.array([.17,-.63,.48,.29,-.37,.23])]:
            for factor in [1.,1.25]:
                g,m=options.fast_terms(study,q,factor);expected=study.terms(q,factor)
                np.testing.assert_allclose(g,expected[0],atol=1e-10)
                np.testing.assert_allclose(m,expected[1],atol=1e-10)
                gradient=[]
                for j in range(6):
                    step=np.eye(6)[j]*1e-6
                    gradient.append((study.terms(q+step,factor)[3]-study.terms(q-step,factor)[3])/2e-6)
                np.testing.assert_allclose(g,gradient,atol=1e-6)

    def test_closed_form_one_joint_pendulum(self):
        study=engineering.Study.__new__(engineering.Study)
        study.origins=np.zeros((6,3));study.axes=np.tile([0.,1.,0.],(6,1));study.reflected=np.zeros(6)
        study.bodies=[('fixture',6.,np.array([.15,0.,0.]),np.zeros((3,3)),[0])]
        g,m=options.fast_terms(study,np.zeros(6))
        self.assertAlmostEqual(g[0],-8.825985)
        self.assertAlmostEqual(m[0,0],.135)
        self.assertAlmostEqual(options.fast_terms(study,np.array([np.pi/2,0,0,0,0,0]))[0][0],0)

    def test_resizing_mass_rotor_and_adapter_bookkeeping(self):
        variant=next(o for o in self.config['options'] if o['id']=='shoulder142_r100')
        study,config=options.build_study(engineering,self.geometry,self.original,self.config,variant,self.materials,'drawing_materials',150,5)
        for index,expected in [(2,8.22),(4,1.75),(5,1.75),(6,1.16)]:
            self.assertAlmostEqual(sum(b[1] for b in study.bodies if b[0].startswith('joint'+str(index)+'-')),expected)
        self.assertAlmostEqual(sum(b[1] for b in study.bodies if b[0].startswith('adapter-')),.75)
        self.assertAlmostEqual(study.reflected[1],1273287*1e-9*101**2)
        self.assertEqual(config['jointCandidates'][5]['family'],'eRob70T')
        # Isolated variants must not mutate baseline input or supplier geometry.
        self.assertEqual(self.original['jointCandidates'][1]['family'],'eRob110T')
        q=np.array([.17,-.63,.48,.29,-.37,.23]);g,m=options.fast_terms(study,q)
        expected=study.terms(q)
        np.testing.assert_allclose(g,expected[0],atol=1e-10)
        np.testing.assert_allclose(m,expected[1],atol=1e-10)
        self.assertGreaterEqual(np.linalg.eigvalsh(m).min(),-1e-10)

    def test_unsupported_tree_rejected(self):
        study=engineering.Study(self.geometry,self.original)
        study.bodies=[('unsupported',1.,np.zeros(3),np.zeros((3,3)),[1])]
        with self.assertRaises(ValueError): options.fast_terms(study,np.zeros(6))

    def test_zero_speed_acceleration_bound(self):
        result=options.acceleration_bound(np.array([[2.]]),np.array([[[1.]]]),np.array([8.]),1.)
        self.assertAlmostEqual(result,np.rad2deg(6.))
        self.assertIsNone(options.acceleration_bound(np.array([[9.]]),np.array([[[1.]]]),np.array([8.]),1.))


if __name__=='__main__': unittest.main()
