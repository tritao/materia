"""Negative controls against a real exported packet, plus unit/result parsing checks."""

import argparse
import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from packet import elastic_input, validate
from solver_fixtures import read_table

PACKET = Path(__file__).resolve().parents[1] / "build/fem/center/manifest.json"


class FemChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.packet = json.loads(PACKET.read_text())

    def mutated(self, change):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "packet"
            shutil.copytree(PACKET.parent, target)
            packet = copy.deepcopy(self.packet)
            change(packet, target)
            manifest = target / "manifest.json"
            manifest.write_text(json.dumps(packet))
            return validate(manifest)

    def test_real_packet(self):
        result = validate(PACKET)
        self.assertGreaterEqual(result["solids"], 30)
        self.assertGreater(result["components"], result["solids"])

    def test_unit_conversion(self):
        material = elastic_input(35e9, .25, 2400)
        self.assertAlmostEqual(material["youngsMPa"], 35000)
        self.assertAlmostEqual(material["densityTonnePerMm3"], 2.4e-9, places=16)

    def test_missing_or_invalid_elastic_input(self):
        for args in ((None, .25, 2400), (35e9, .5, 2400), (35e9, .25, float("nan")),
                     (-35e9, .25, 2400), (True, .25, 2400)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                elastic_input(*args)

    def test_wrong_units(self):
        with self.assertRaisesRegex(ValueError, "units"):
            self.mutated(lambda p, _: p["units"].update(mass="tonne"))

    def test_double_pose_application(self):
        with self.assertRaisesRegex(ValueError, "world coordinates"):
            self.mutated(lambda p, _: p.update(poseApplication="apply-pose"))

    def test_missing_structural_component(self):
        with self.assertRaisesRegex(ValueError, "Missing required component"):
            self.mutated(lambda p, _: p.update(parts=[x for x in p["parts"] if x["id"] != "xyStage/table"]))

    def test_duplicate_component(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.mutated(lambda p, _: p["parts"].append(p["parts"][0]))

    def test_changed_step_file(self):
        def corrupt(packet, folder):
            part = next(p for p in packet["parts"] if p["representation"] == "solid")
            (folder / part["stepFile"]).write_text("not the reviewed solid")
        with self.assertRaisesRegex(ValueError, "digest mismatch"):
            self.mutated(corrupt)

    def test_path_escape(self):
        def corrupt(packet, _):
            part = next(p for p in packet["parts"] if p["representation"] == "solid")
            part["stepFile"] = "../unreviewed.step"
        with self.assertRaisesRegex(ValueError, "relative filename"):
            self.mutated(corrupt)

    def test_payload_offset(self):
        with self.assertRaisesRegex(ValueError, "50 mm above"):
            self.mutated(lambda p, _: p["datums"]["workpieceReferenceMm"].update(z=0))

    def test_result_parser(self):
        text = "displacements (vx,vy,vz) for set TIP and time 1\n\n9 1.0D-3 0 0\n\n"
        self.assertEqual(read_table(text, "U"), {9: (.001, 0, 0)})
        for invalid in ("no results", text + text, text.replace("\n\n", "\n9 0 0 0\n")):
            with self.subTest(text=invalid), self.assertRaises(ValueError):
                read_table(invalid, "U")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, default=PACKET)
    args, remaining = parser.parse_known_args()
    PACKET = args.packet.resolve()
    unittest.main(argv=[__file__] + remaining)
