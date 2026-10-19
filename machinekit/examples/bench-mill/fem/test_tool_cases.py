"""Known machine/tool split verifies the casting decision cannot be tool-masked."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from compliance_budget import tool_case_packet
from packet import digest
from tool_cases import collect

class ToolCaseChecks(unittest.TestCase):
    def check_report(self,root,mixed=False,threads=1):
        cases=[{'id':'test-bar-20','decisionCase':True,'holderLengthMm':0,'stickOutMm':40},
               {'id':'cutter-6-steel','decisionCase':False,'holderLengthMm':35,'stickOutMm':20}]
        inputs={'toolCases':cases,'toolHolder':{'neckLengthMm':35}}
        config={'cuttingForceN':500,'frameShareTarget':{'maximumFraction':.3}}
        inputs_path=root/'inputs.json';inputs_path.write_text(json.dumps(inputs))
        config_path=root/'config.json';config_path.write_text(json.dumps(config))
        packet={'datums':{'toolTipMm':{'x':0,'y':0,'z':0}},'parts':[{'id':'spindle/holder','boundsMm':{'minZ':100}}]}
        manifest=root/'manifest.json';manifest.write_text(json.dumps(packet))
        for i,case in enumerate(cases):
            directory=root/case['id'];directory.mkdir()
            report={'converged':True,'solverThreads':threads,'manifestSha256':('different-packet' if mixed and i else digest(manifest)),
                'frozenManifestPath':str(manifest),'budgetConfigurationSha256':digest(inputs_path),'configurationSha256':digest(config_path),
                'analysisContactMm':tool_case_packet(packet,inputs,case['id'])['datums']['toolTipMm'],
                'meshes':[{'supports':{'soft-three-point':{'solution':{}}}}]}
            (directory/'report.json').write_text(json.dumps(report))
        diagonal=lambda value:[[value if i==j else 0 for j in range(3)] for i in range(3)]
        # Total 100; job tool 90; casting 3.1. The machine share is 31%,
        # despite the tool-masked total-loop share being only 3.1%.
        known={'componentComplianceUmPerN':{'tool':diagonal(90)},'complianceUmPerN':diagonal(100),
            'frameEnergyDiagonalUmPerN':{a:{'BASE_CASTING':3.1,'COLUMN_CASTING':0} for a in 'xyz'}}
        with patch('tool_cases.budget',side_effect=lambda *args:copy.deepcopy(known)):
            return collect(root,inputs_path,config_path)

    def test_target_uses_machine_loop_and_context_cannot_decide(self):
        with tempfile.TemporaryDirectory() as folder:
            report=self.check_report(Path(folder))
        self.assertTrue(report['castingShareReachesTarget'])
        decision=report['cases'][0]['supports']['soft-three-point']
        context=report['cases'][1]['supports']['soft-three-point']
        self.assertAlmostEqual(decision['castingMachineLoopShares']['x'],.31)
        self.assertTrue(decision['geometryEligible'])
        self.assertIsNone(context['geometryEligible'])

    def test_parallel_reports_cannot_qualify_budget(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'qualified serial'):
                self.check_report(Path(folder),threads=4)

    def test_mixed_cad_packets_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'same frozen CAD packet'):
                self.check_report(Path(folder),True)

if __name__=='__main__':unittest.main()
