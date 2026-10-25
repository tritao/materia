import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lcsccable as lcsc
import cable_assembly as cables


class CableCaptureTests(unittest.TestCase):
    def test_literal_decoder_does_not_execute_code(self):
        self.assertEqual(lcsc.literal_array('x=[{label:"a:b",value:"a]b",ok:!0}]', 'x='),
                         [{'label':'a:b','value':'a]b','ok':True}])
        with self.assertRaises(json.JSONDecodeError):
            lcsc.literal_array('x=[process.exit()]', 'x=')

    def test_script_capture_ignores_external_trackers(self):
        parser = lcsc.Scripts()
        parser.feed('<script src="//static.lcsccable.com/a.js"></script><script src="https://tracker.test/b.js"></script>')
        self.assertEqual(parser.urls, ['https://static.lcsccable.com/a.js'])

    def test_cache_offline_and_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            client = lcsc.Client(Path(directory), offline=True)
            url = lcsc.BASE + '/customize'
            with self.assertRaisesRegex(ValueError, 'Missing cached'):
                client.get(url)
            path = Path(directory)/(hashlib.sha256(url.encode()).hexdigest()+'.json')
            lcsc.write_json(path, {'url':url,'body':'modified','sha256':'incorrect'})
            with self.assertRaisesRegex(ValueError, 'Corrupt'):
                client.get(url)

    def test_snapshot_coverage_preserves_repeated_style_names(self):
        catalog = {'coverage':{},'parts':[], 'wire_styles':[
            {'id':'a','raw':{'productMpn':'UL3239','displayMpn':'UL3239(3KV)','voltage':'3000'},'missing':['od']},
            {'id':'b','raw':{'productMpn':'UL3239','displayMpn':'UL3239(6KV)','voltage':'6000'},'missing':['od']}]}
        matrix = lcsc.matrix(catalog)
        self.assertEqual(len(matrix['items']),2)
        self.assertEqual(len(matrix['repeated_style_designations']['UL3239']),2)
        self.assertEqual(matrix['manufacturing_qualified_count'],0)

    def test_capture_failure_does_not_claim_complete_selectors(self):
        base = lcsc.BASE
        quote = ('tabOptions:[{label:"Consumer Electronics",value:"consumer_electronics",quote:!0,'
                 'options:[{label:"Terminal Harness",value:"terminal_wire"}]}],'
                 'shapeOptions:[{label:"Single",value:"single_head_harness"}],'
                 'he=[{value:10,label:"one"},{value:20,label:"two"},{value:30,label:"three"},{value:40,label:"other"}],me=')
        shared = ('i=[{value:"black",label:"Black"},{value:"multiColor",label:"MultiColor"}],'
                  'a=[{label:"Flat cut",value:"flat_cutting"}],c=[{label:"mm",value:"mm"}]')
        def record(url, body, status=200):
            return {'url':url,'body':body,'status':status,'fetched_at':'fixture',
                    'sha256':hashlib.sha256(body.encode()).hexdigest()}
        class FixtureClient:
            def get(self, url):
                if url == base+'/customize':
                    return record(url, ''.join('<script src="//static.lcsccable.com/'+name+'.js"></script>' for name in ['quote','shared','rules']))
                for name, body in [('quote',quote),('shared',shared),('rules','Minimum length is 30mm')]:
                    if url.endswith('/'+name+'.js'): return record(url,body)
                if url == base+lcsc.ENDPOINTS['wire_styles']:
                    return record(url,'<html>Access denied</html>',403)
                items = ['AWG24'] if 'returnMpnType=cable' in url else ['JST_XHP-2_C144401'] if 'rubber_shell' in url else ['JST_SXH-001T-P0.6_C140573']
                return record(url,json.dumps({'code':200,'result':items}))
        catalog = lcsc.capture(FixtureClient())
        self.assertFalse(catalog['coverage']['selector_lists_complete'])
        self.assertEqual(catalog['coverage']['api']['wire_styles']['source']['status'],403)
        self.assertEqual(catalog['coverage']['api']['wire_styles']['status'],'unavailable')
        self.assertEqual(catalog['wire_styles'],[])
        self.assertEqual(len(catalog['parts']),2)
        self.assertEqual(catalog['families'][0]['configuration'],'online_terminal_harness')

    def test_unknown_identity_is_rejected(self):
        with self.assertRaises(ValueError): lcsc.part('XHP-2','housing',{})
        self.assertEqual(lcsc.part('JST_XHP-2_C144401','housing',{})['mpn'],'XHP-2')


class CableRecipeTests(unittest.TestCase):
    def setUp(self):
        self.recipe = json.loads(cables.EXAMPLE.read_text())
        self.reference = json.loads(cables.REFERENCE.read_text())

    def validate(self): return cables.validate(self.recipe, self.reference)

    def test_reference_bom_connections_and_unknowns(self):
        report = self.validate()
        self.assertFalse(report['manufacturing_qualified'])
        counts = {r['supplier_part_id']:r['quantity'] for r in report['bom'] if 'supplier_part_id' in r}
        self.assertEqual(counts, {'C144401':2,'C140573':4})
        self.assertEqual([c['routed_length_mm'] for c in report['connections']], [200,200])
        self.assertTrue(any('exact manufacturer MPN unknown' in s for s in report['open_requirements']))
        self.assertTrue(any('strip_length_mm: unknown' in s for s in report['open_requirements']))
        self.assertEqual(self.validate(),report)

    def test_incompatible_wire_gauge(self):
        self.recipe['wire_types'][0]['awg']=30
        with self.assertRaisesRegex(ValueError,'wire gauge'): self.validate()

    def test_incompatible_insulation_od(self):
        self.recipe['wire_types'][0]['outer_diameter_mm']['value']=2.0
        with self.assertRaisesRegex(ValueError,'insulation diameter'): self.validate()

    def test_wrong_contact(self):
        self.recipe['wires'][0]['from']['contact_id']='C111515'
        with self.assertRaisesRegex(ValueError,'Contact incompatible'): self.validate()

    def test_duplicate_cavity(self):
        self.recipe['wires'][1]['from']['cavity']=1
        with self.assertRaisesRegex(ValueError,'Duplicate'): self.validate()

    def test_missing_cavity(self):
        self.recipe['wires'].pop()
        with self.assertRaisesRegex(ValueError,'Every cavity'): self.validate()
        for end in self.recipe['ends']: end['unpopulated_cavities']=[2]
        self.assertEqual(len(self.validate()['connections']),1)

    def test_short_cut_length(self):
        self.recipe['wires'][0]['cut_length_mm']=209
        with self.assertRaisesRegex(ValueError,'shorter than route'): self.validate()

    def test_unrouted_slack_is_reported(self):
        self.recipe['wires'][0]['cut_length_mm']=220
        report=self.validate()
        self.assertEqual(report['connections'][0]['unrouted_slack_mm'],10)
        self.assertTrue(any('slack' in s for s in report['open_requirements']))

    def test_route_must_meet_cavity(self):
        self.recipe['wires'][0]['route'][0][0]+=1
        with self.assertRaises(ValueError): self.validate()

    def test_reversed_exit(self):
        self.recipe['ends'][0]['exit_direction']=[0,-1,0]
        self.recipe['ends'][0]['up']=[0,0,-1]
        with self.assertRaisesRegex(ValueError,'exit direction'): self.validate()

    def test_corners_are_not_misreported_as_bend_safe(self):
        self.recipe['wires'][0]['route'].insert(1,[-1.25,100,10])
        with self.assertRaisesRegex(ValueError,'Sharp route corners'): self.validate()

    def test_nonfinite_and_boolean_dimensions_rejected(self):
        for value in [float('nan'),float('inf'),True,-1]:
            self.recipe['wire_types'][0]['outer_diameter_mm']['value']=value
            with self.assertRaises(ValueError): self.validate()

    def test_tinning_requires_strip_length(self):
        self.recipe['wires'][0]['from']['tinning_length_mm']=2
        with self.assertRaisesRegex(ValueError,'Tinning exceeds'): self.validate()

    def test_overlapping_end_stripping_is_rejected(self):
        for key in ('from','to'):
            self.recipe['wires'][0][key]['strip_length_mm']={'value':110,'status':'design_assumption','evidence':'test'}
        with self.assertRaisesRegex(ValueError,'Combined stripping'): self.validate()

    def test_report_preserves_recipe_and_evidence_fingerprints(self):
        report=self.validate()
        self.assertEqual(report['recipe'],self.recipe)
        self.assertEqual(len(report['input_fingerprints']['recipe_sha256']),64)
        self.assertEqual(set(report['part_evidence']),{'C144401','C140573'})
        self.recipe['wires'][0]['cut_length_mm']=211
        self.assertNotEqual(report['input_fingerprints']['recipe_sha256'],self.validate()['input_fingerprints']['recipe_sha256'])

    def test_supplier_membership_and_full_style_identity(self):
        catalog={'parts':[{'id':r['id']} for r in self.reference['parts']],
                 'gauge_options':['AWG24'], 'wire_styles':[{'raw':self.recipe['wire_types'][0]['supplier_style']}]}
        self.assertEqual(len(cables.validate(self.recipe,self.reference,catalog)['connections']),2)
        bad=copy.deepcopy(catalog);bad['wire_styles'][0]['raw']['voltage']='600'
        with self.assertRaisesRegex(ValueError,'exact supplier record'):
            cables.validate(self.recipe,self.reference,bad)
        bad=copy.deepcopy(catalog);bad['parts']=[p for p in bad['parts'] if p['id']!='C140573']
        with self.assertRaisesRegex(ValueError,'Contact missing'):
            cables.validate(self.recipe,self.reference,bad)

    def test_documents_and_geometry_do_not_imply_current_rating(self):
        report=self.validate()
        self.assertNotIn('current_a',report)
        self.assertEqual(self.reference['family_ratings']['current_a']['condition'],
                         'AWG22 only; not assigned to AWG24 example')
        self.assertEqual(len(self.reference['parts']),20)
        for part in self.reference['parts']:
            self.assertEqual(part['source']['page'],2)
            self.assertEqual(len(part['source']['sha256']),64)


if __name__=='__main__': unittest.main()
