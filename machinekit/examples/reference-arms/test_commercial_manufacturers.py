import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('commercial',ROOT/'commercial-reference-study.py')
commercial=importlib.util.module_from_spec(spec);spec.loader.exec_module(commercial)
design=commercial.design_module()


class ManufacturerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.data=json.loads((ROOT/'build/manufacturer-validation/inputs.json').read_text())

    def test_fr5_zero_pose_hand_fixture(self):
        oracle=self.data['models'][1]['oracle'];actual=design.dh_fk([0]*6,oracle)
        np.testing.assert_allclose(actual[:3,3],[-.82,-.352,.05],atol=1e-14)
        np.testing.assert_allclose(actual[:3,:3],[[1,0,0],[0,0,-1],[0,1,0]],atol=1e-14)
        actual=design.dh_fk([math.pi/2,0,0,0,0,0],oracle)
        np.testing.assert_allclose(actual[:3,3],[.352,-.82,.05],atol=1e-14)

    def test_precision_bound_does_not_hide_wrong_link(self):
        oracle=self.data['models'][1]['oracle'];wrong=copy.deepcopy(oracle);wrong['aMetres'][2]+=.01
        manifest=json.loads((ROOT/'build/fairino5_v6/manifest.json').read_text());bound=commercial.precision_bound(manifest)
        self.assertLess(bound['translationBoundM'],.00012)
        error=np.linalg.norm(design.dh_fk([0]*6,oracle)[:3,3]-design.dh_fk([0]*6,wrong)[:3,3])
        self.assertGreater(error,80*bound['translationBoundM'])

    def test_native_against_frozen_manufacturer_golden_matrices(self):
        native=json.loads((ROOT/'build/manufacturer-validation/native.json').read_text())
        fixtures=json.loads((ROOT/'manufacturer-fk-goldens.json').read_text())['fixtures']
        for fixture in fixtures:
            model=next(m for m in native['models'] if m['id']==fixture['model'])
            observation=next(o for o in model['observations'] if o['q']==fixture['q'])
            actual=design.frame(observation['tcp']);expected=np.array(fixture['tcpMatrix'])
            if fixture['model']=='fairino5_v6':
                manifest=json.loads((ROOT/'build/fairino5_v6/manifest.json').read_text());bound=commercial.precision_bound(manifest)
            else:bound=dict(translationBoundM=1e-8,rotationOperatorBound=1e-8)
            self.assertLessEqual(np.linalg.norm(actual[:3,3]-expected[:3,3]),bound['translationBoundM'])
            self.assertLessEqual(np.linalg.norm(actual[:3,:3]-expected[:3,:3],2),bound['rotationOperatorBound'])


if __name__=='__main__':unittest.main()
