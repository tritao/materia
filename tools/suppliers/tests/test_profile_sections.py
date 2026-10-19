from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]))
import profile_sections


def dxf(vertices, closed=True):
    pairs=[(0,"SECTION"),(2,"ENTITIES"),(0,"POLYLINE"),(70,1 if closed else 0)]
    for x,y,z,bulge in vertices:
        pairs += [(0,"VERTEX"),(10,x),(20,y),(30,z),(42,bulge)]
    pairs += [(0,"SEQEND"),(0,"ENDSEC"),(0,"EOF")]
    return "\n".join(str(v) for pair in pairs for v in pair).encode()


class SectionTests(unittest.TestCase):
    def test_shared_profile_family_layouts(self):
        expected=[((30,30),"TSLOT-30-SLOT8",4,1),((40,40),"TSLOT-40-SLOT8",4,1),
                  ((40,80),"TSLOT-40-SLOT8",6,2),((80,80),"TSLOT-40-SLOT8",8,4)]
        for dimensions,system,slot_count,bore_count in expected:
            family=profile_sections.profile_family(dimensions)
            self.assertEqual(family["system_id"],system)
            self.assertEqual(sum(map(len,family["slot_positions_mm"])),slot_count)
            self.assertEqual(len(family["bore_centres_mm"]),bore_count)
            self.assertNotIn("supplier_id",family)
            self.assertNotIn("sku",family)
            self.assertEqual(family["interface_status"],"candidate_not_cross_vendor_qualified")
        plate=profile_sections.profile_family((180,20))
        self.assertEqual(plate["kind"],"slotted_plate")
        self.assertIsNone(plate["system_id"])

    def test_translated_elevated_section_normalizes_to_xy(self):
        result=profile_sections.parse(dxf([(10,20,2000,0),(40,20,2000,0),(40,50,2000,0),(10,50,2000,0)]))
        self.assertEqual(result["vertex_extents_mm"],[30,30])
        self.assertEqual(result["source_elevation_mm"],2000)
        self.assertAlmostEqual(result["area_mm2"],900)
        self.assertEqual(result["loops"][0][0],[-15,-15,0])

    def test_bulge_arc_area_and_reversal(self):
        vertices=[[1,0,math.tan(math.pi/8)],[0,1,0],[0,0,0]]
        self.assertAlmostEqual(profile_sections.area(vertices),math.pi/4)
        self.assertAlmostEqual(profile_sections.area(profile_sections.reverse(vertices)),-math.pi/4)

    def test_open_or_nonplanar_section_rejected(self):
        vertices=[(0,0,0,0),(1,0,0,0),(1,1,1,0),(0,1,0,0)]
        for content in [dxf(vertices),dxf(vertices,False)]:
            with self.assertRaises(ValueError):
                profile_sections.parse(content)


if __name__=="__main__":
    unittest.main()
