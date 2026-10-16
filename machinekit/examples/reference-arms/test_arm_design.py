"""Analytical fixtures for the independent arm-design oracle and Jacobian checks."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('arm_design',ROOT/'arm-design-study.py')
design=importlib.util.module_from_spec(spec);spec.loader.exec_module(design)


class ArmDesignChecks(unittest.TestCase):
    def test_manufacturer_zero_pose_includes_base_convention_and_rotated_tcp(self):
        oracle=json.loads((ROOT/'arm-reference-oracles.json').read_text())['ur5e']
        actual=design.dh_fk([0.]*6,oracle)
        np.testing.assert_allclose(actual[:3,3],[.8172,.3829,.0628],atol=1e-14)
        np.testing.assert_allclose(actual[:3,:3],[[-1,0,0],[0,0,1],[0,1,0]],atol=1e-14)

    def test_wrong_forearm_length_is_detectable_independently_of_ik_roundtrips(self):
        oracle=json.loads((ROOT/'arm-reference-oracles.json').read_text())['ur5e']
        wrong=json.loads(json.dumps(oracle));wrong['aMetres'][2]+=.01
        q=[.2,-.8,1.,.1,-.6,.2]
        error=np.linalg.norm(design.dh_fk(q,oracle)[:3,3]-design.dh_fk(q,wrong)[:3,3])
        self.assertAlmostEqual(error,.01,places=12)
        self.assertGreater(error,oracle['translationToleranceM'])

    def test_xml_origin_translation_is_not_rotated_by_its_own_rpy(self):
        text='''<robot name="fixture"><joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
        <joint name="axis" type="revolute"><parent link="base"/><child link="tip"/>
        <origin xyz="2 0 0"/><axis xyz="0 0 1"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'robot.urdf';path.write_text(text)
            actual=design.urdf_fk(path,'tip',np.eye(4))([math.pi/2])
        np.testing.assert_allclose(actual[:3,3],[1,4,3.15],atol=1e-14)
        np.testing.assert_allclose(actual[:3,:3],np.diag([-1,-1,1]),atol=1e-14)

    def test_finite_difference_jacobian_matches_a_two_link_analytical_fixture(self):
        def fk(q):
            a,b=q[:2];m=np.eye(4)
            m[:3,:3]=design.rotation([0,0,1],a+b)
            m[:3,3]=[.3*math.cos(a)+.2*math.cos(a+b),.3*math.sin(a)+.2*math.sin(a+b),0]
            return m
        a,b=.4,-.7;q=[a,b,0,0,0,0];expected=np.zeros((6,6))
        expected[0,:2]=[-.3*math.sin(a)-.2*math.sin(a+b),-.2*math.sin(a+b)]
        expected[1,:2]=[.3*math.cos(a)+.2*math.cos(a+b),.2*math.cos(a+b)]
        expected[5,:2]=[1,1]
        np.testing.assert_allclose(design.numeric_jacobian(fk,q),expected,atol=1e-9)


if __name__=='__main__':unittest.main()
