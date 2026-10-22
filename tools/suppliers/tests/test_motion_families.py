import copy
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]))
import hg
import supports
import sfu_family


def fixture():
    products={};variants=[];assets=[]
    for index,m in enumerate(hg.models()+supports.models()+sfu_family.models()):
        pid=m['parent_id'];p=products.setdefault(pid,dict(supplier_product_id=pid,url=m['source_url'],specification_tables=[[]]))
        p['specification_tables'][0].append(copy.deepcopy(m['table_row']))
        if pid==39276:
            if m['model'][3:5] in ('35','45'):continue
            sku=m['table_row'][-1];vid=index+1;options=[]
        elif pid in (3436,3468):
            if m['model'][3:5] in ('35','45'):continue
            sku='block-'+m['model'];vid=index+1;options=[dict(value=m['table_row'][0].lower())]
        else:sku=m['sku'];vid=m['variant_id'];options=[]
        variants.append(dict(parent_id=pid,sku=sku,supplier_product_id=vid,option_attributes=options))
        if pid in (2348,2378,2236,2266):assets.append(dict(sha256=m['sha256'],product_id=pid,variant_ids=[vid],usable_geometry=True))
    # Stock screw rows are published standard-end options, length stays a design input.
    products[3000]=dict(supplier_product_id=3000,url='stock',specification_tables=[[]])
    for index,(model,sku) in enumerate([('SFU2005','1033003'),('SFU2010','1033004'),('SFU2505','1033005'),('SFU2510','1033006')]):
        products[3000]['specification_tables'][0].append([model,sku,model[-2:].lstrip('0'),model[3:5]])
        variants.append(dict(parent_id=3000,sku=sku,supplier_product_id=9000+index))
    return dict(products=list(products.values()),variants=variants),assets


class MotionFamilyTests(unittest.TestCase):
    def test_generated_sources_are_current(self):
        for module in (hg,supports,sfu_family):self.assertEqual(module.TARGET.read_text(),module.generated())

    def test_hg_only_actual_supplier_variants(self):
        c,_=fixture();r=hg.build(c)['recipes']
        self.assertEqual(len(r),12)
        self.assertEqual(sum(x['geometry_source']=='reference_cad' for x in r),8)
        self.assertFalse(any(x['values']['profile'] in ('HGH35CA','HGW45CC') for x in r))
        self.assertTrue(all(x['required_inputs']==['length'] for x in r if x['geometry_source']=='generator'))

    def test_changed_hg_evidence_fails(self):
        c,_=fixture();next(p for p in c['products'] if p['supplier_product_id']==3436)['specification_tables'][0][0][1]='99'
        with self.assertRaises(ValueError):hg.build(c)

    def test_support_quarantine_and_pinned_cad(self):
        c,a=fixture();r=supports.build(c,a)['recipes']
        self.assertEqual(len(r),20)
        self.assertFalse(any(x['values']['model'] in ('FK17','FF17') for x in r))
        a[0]['sha256']='changed'
        self.assertEqual(len(supports.build(c,a)['recipes']),19)
        checks=json.loads(supports.MANIFEST.read_text())['interface_checks']
        self.assertEqual([x['model'] for x in checks if not x['mounting_pattern_passes']],['FK17','FF17'])

    def test_sfu_does_not_invent_thread_depth_or_root(self):
        c,_=fixture();r=sfu_family.build(c)['recipes']
        self.assertEqual(len(r),13)
        for x in r:
            self.assertTrue(x['required_inputs'])
            self.assertNotIn('raceRootDiameter',x['values'])
        screws=[x for x in r if x['parent_id']==3000]
        self.assertEqual(len(screws),4)
        self.assertTrue(all(not x['custom_end_machining'] for x in screws))

    def test_sfu_table_change_is_rejected(self):
        c,_=fixture();next(p for p in c['products'] if p['supplier_product_id']==3097)['specification_tables'][0][0][2]='99'
        with self.assertRaises(ValueError):sfu_family.build(c)
