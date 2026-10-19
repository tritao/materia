import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hzmotion

class HzMotionTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(hzmotion.REFERENCE.read_text())
    def test_pinned_runtime(self):
        self.assertEqual(hzmotion.render(hzmotion.validate(self.data)), hzmotion.TARGET.read_text())
    def test_no_fabricated_cad_assets(self):
        for item in self.data['cadDownloads']:
            self.assertEqual(item['status'], 403)
            self.assertNotIn('sha256', item)
    def test_wrong_bearing_interface_is_rejected(self):
        next(p for p in self.data['parts'] if p['model']=='BF12')['shaftBore']=12
        with self.assertRaisesRegex(ValueError,'floating journal'):
            hzmotion.validate(self.data)
    def test_geometry_record_cannot_silently_lose_source(self):
        self.data['parts'][0]['page']=999
        with self.assertRaisesRegex(ValueError,'drawing page'):
            hzmotion.validate(self.data)
    def test_coverage_does_not_claim_small_z_c_mount(self):
        self.assertFalse(any(r['family']=='C' and r['motorInterface']=='80E' and r['support']=='BK12' for r in self.data['coverage']))
        for family in ['A','B','C','H','L','KK']:
            self.assertTrue(any(r['family']==family for r in self.data['coverage']))
    def test_integrated_parts_keep_bearing_and_locking_scope(self):
        for p in self.data['parts']:
            if p['family']=='C':
                self.assertEqual(p['bearingArrangement'],'DF')
                self.assertIn('internal hardware',p['runningMassIncluded'])

if __name__=='__main__':unittest.main()
