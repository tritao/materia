import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parents[1]))
spec=importlib.util.spec_from_file_location('supplier_coverage',Path(__file__).parents[1]/'coverage.py')
coverage=importlib.util.module_from_spec(spec);spec.loader.exec_module(coverage)


class CoverageRecipeTests(unittest.TestCase):
    def test_supplier_cad_and_reference_recipes_remain_usable(self):
        products=[dict(supplier_product_id=pid,url='test',specification_tables=[[['B','H','h']]]) for pid in (2348,2378,3436)]
        variants=[dict(parent_id=pid,supplier_product_id=pid+1,sku=str(pid)) for pid in (2348,2378,3436)]
        recipes=[dict(parent_id=pid,supplier_product_id=pid+1,sku=str(pid),type_id='test.component',geometry_source=source,values={},required_inputs=[]) for pid,source in [(2348,'supplier_cad'),(2378,'generator'),(3436,'reference_cad')]]
        audit=[dict(supplier_product_id=pid,name='test',url='test',status='geometry_gap',candidate_families=[]) for pid in (2348,2378,3436)]
        with tempfile.TemporaryDirectory() as directory,contextlib.ExitStack() as stack:
            output=Path(directory);(output/'cad').mkdir()
            for name,value in {'catalog.json':dict(products=products,variants=variants),'audit.json':dict(products=audit),
                'cad/inventory.json':dict(downloads=[],complete=True),'cad/validation.json':dict(assets=[],complete=True),
                'fasteners.json':dict(mappings=[])}.items():(output/name).write_text(json.dumps(value))
            for name,value in [('hardware_review',dict(recipes=[])),('motor_mount_review',dict(recipes=[],bindings=[])),
                ('sfu16_review',dict(recipes=recipes)),('hg_review',dict(recipes=[])),('supports_review',dict(recipes=[])),('sfu_family_review',dict(recipes=[]))]:
                stack.enter_context(patch.object(coverage,name,return_value=value))
            with contextlib.redirect_stdout(io.StringIO()):coverage.build(output)
            result=json.loads((output/'coverage.json').read_text())
            p={p['product_id']:p for p in result['products']}
            self.assertEqual(p[2348]['status'],'supplier_component_available')
            self.assertEqual(p[2378]['status'],'generator_available')
            self.assertEqual(p[3436]['variants'][0]['geometry_source'],'reference_cad')
            self.assertTrue(all(p[pid]['variants'][0]['recipe_available'] for pid in p))
            self.assertEqual(p[2348]['generator_skus'],[])
            self.assertEqual(p[2348]['component_skus'],['2348'])
            self.assertEqual(p[2348]['variants'][0]['component_type_id'],'test.component')
