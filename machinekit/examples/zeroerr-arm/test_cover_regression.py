"""The collision reuse gate must reject geometry/schema changes and truncated files."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('regression', Path(__file__).with_name('review-cover-regression.py'))
regression = importlib.util.module_from_spec(spec)
spec.loader.exec_module(regression)


class StepReuseTests(unittest.TestCase):
    STEP = (b"ISO-10303-21;\nHEADER;\nFILE_NAME ('a.STEP','old',(),(),'writer','cad','');\n"
            b"FILE_SCHEMA(('AP214'));\nENDSEC;\nDATA;\n#1=CARTESIAN_POINT('',(1.,2.,3.));\n"
            b"ENDSEC;\nEND-ISO-10303-21;\n")

    def test_export_timestamp_is_metadata(self):
        other = self.STEP.replace(b"'a.STEP','old'", b"'b.STEP','new'")
        self.assertEqual(regression.normalized_step(self.STEP), regression.normalized_step(other))

    def test_geometry_edit_is_not_reusable(self):
        other = self.STEP.replace(b'(1.,2.,3.)', b'(1.,2.,4.)')
        self.assertNotEqual(regression.normalized_step(self.STEP), regression.normalized_step(other))

    def test_schema_edit_is_not_reusable(self):
        self.assertNotEqual(regression.normalized_step(self.STEP),
                            regression.normalized_step(self.STEP.replace(b'AP214', b'AP242')))

    def test_truncated_or_multiple_sections_fail(self):
        for data in (self.STEP[:-25], self.STEP.replace(b'\nDATA;', b'\nDATA;\nDATA;')):
            with self.assertRaises(ValueError):
                regression.normalized_step(data)


if __name__ == '__main__':
    unittest.main()
