"""Run with the preparation venv: python -m unittest discover -s .../reference-arms."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import prepare


class ImportChecks(unittest.TestCase):
    def test_urdf_rpy_is_extrinsic_xyz_and_translation_is_not_rotated(self):
        element = ET.fromstring('<origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>')
        matrix = prepare.origin(element)
        np.testing.assert_allclose(matrix @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-14)
        frame = prepare.frame(matrix)
        self.assertAlmostEqual(frame["qz"], np.sqrt(0.5))
        self.assertAlmostEqual(frame["qw"], np.sqrt(0.5))

    def test_mesh_path_cannot_escape_the_declared_package(self):
        package = Path("/tmp/reference-test/package")
        with self.assertRaises(ValueError):
            prepare.mesh_relative("package://description/../../secret.stl", package, "description")
        with self.assertRaises(ValueError):
            prepare.mesh_relative("https://example.com/model.stl", package, "description")
        self.assertEqual(prepare.mesh_relative("../meshes/a.stl", package, "description"), "meshes/a.stl")

    def test_changed_vendor_source_is_rejected_before_publishing(self):
        with patch.object(prepare, "description", return_value=(None, "", Path("/tmp"), {"changed": "hash"})):
            with self.assertRaisesRegex(ValueError, "Pinned source file hashes changed"):
                prepare.compile_model("ur", "ur5e")

    def test_nonfinite_and_malformed_coordinates_are_rejected(self):
        for text in ("0 nan 0", "0 inf 0", "0 0"):
            with self.assertRaises(ValueError):
                prepare.numbers(text, 3)

    def test_wire_fields_come_from_consumer_schema(self):
        value = {"schema": 1, "id": "test", "family": "ur", "root": "base", "tip": "tip",
                 "tipStatus": "test", "metresPerUnit": 1, "urdfSha256": "hash",
                 "links": [], "joints": [], "probes": []}
        encoded = prepare.wire_manifest(value)
        self.assertEqual(encoded["version"], 1)
        fields = dict(encoded["value"])
        self.assertEqual(fields["1:schema"], 1)
        self.assertEqual(fields["11:probes"], [])
        self.assertEqual(len(fields), len(value))


if __name__ == "__main__":
    unittest.main()
