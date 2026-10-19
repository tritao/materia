"""Known matrix and spring-platform answers for compliance reduction."""

import math
import unittest

from compliance import reduce
from compliance_budget import network, tool_case_packet
from frame_solver import cross, distribute_wrench


class ComplianceChecks(unittest.TestCase):
    def test_rotated_principal_compliances_and_load_box(self):
        result=reduce([[2,1,0],[1,2,0],[0,0,.5]],500)
        self.assertEqual([round(m['complianceUmPerN'],10) for m in result['principalCompliances']],[3,1,.5])
        direction=result['principalCompliances'][0]['worldDirection']
        self.assertAlmostEqual(abs(direction[0]),math.sqrt(.5))
        self.assertAlmostEqual(abs(direction[1]),math.sqrt(.5))
        self.assertEqual(len(result['loadBoxCorners']),8)
        self.assertAlmostEqual(result['worstLoadBoxCorner']['normUm'],math.sqrt(1500**2*2+250**2))

    def test_tool_case_moves_contact_without_changing_masses(self):
        import json
        from pathlib import Path
        inputs=json.loads(Path(__file__).with_name('compliance-budget.json').read_text())
        packet={'datums':{'toolTipMm':{'x':1,'y':2,'z':100}},'parts':[
            {'id':'spindle/holder','boundsMm':{'minZ':200},'massKg':3}]}
        nose=tool_case_packet(packet,inputs,'spindle-nose')
        bar=tool_case_packet(packet,inputs,'test-bar-20')
        cutter=tool_case_packet(packet,inputs,'cutter-6-carbide')
        self.assertEqual(nose['datums']['toolTipMm']['z'],235)
        self.assertEqual(bar['datums']['toolTipMm']['z'],195)
        self.assertEqual(cutter['datums']['toolTipMm']['z'],180)
        self.assertEqual(packet['datums']['toolTipMm']['z'],100)
        self.assertEqual(nose['parts'],packet['parts'])
        self.assertTrue(nose['analysisToolCase']['decisionCase'])
        self.assertFalse(cutter['analysisToolCase']['decisionCase'])

    def test_nonreciprocal_compliance_fails(self):
        with self.assertRaisesRegex(ValueError,'symmetry'):
            reduce([[1,.1,0],[0,1,0],[0,0,1]],500)

    def test_spring_platform_with_known_overhang(self):
        springs=[]
        for x in (-10,10):
            for y in (-10,10):
                for i in range(3):
                    springs.append(('platform',(x,y,0),[float(i==j) for j in range(3)],1000))
        matrix=network(springs,(0,0,100))[0]['platform']
        self.assertAlmostEqual(matrix[0][0],25.25)
        self.assertAlmostEqual(matrix[1][1],25.25)
        self.assertAlmostEqual(matrix[2][2],.25)

    def test_distributing_mass_transfer_conserves_wrench(self):
        nodes={1:(-2,-3,0),2:(2,-3,0),3:(2,3,0),4:(-2,3,0)}
        point=(5,7,11);force=(13,17,-19)
        loads=distribute_wrench(nodes,nodes,point,force)
        resultant=[0.0]*3;moment=[0.0]*3
        for n,dof,value in loads:
            f=[0.0]*3;f[dof-1]=value
            torque=cross(nodes[n],f)
            for i in range(3):
                resultant[i]+=f[i];moment[i]+=torque[i]
        for actual,expected in zip(resultant+moment,list(force)+list(cross(point,force))):
            self.assertAlmostEqual(actual,expected,places=10)


if __name__=='__main__':
    unittest.main()
