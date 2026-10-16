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

    def test_embedded_material_regions_preserve_colors_and_transforms(self):
        scene = prepare.trimesh.Scene()
        for index, color in enumerate(([125, 173, 204, 255], [8, 8, 8, 255])):
            mesh = prepare.trimesh.creation.box()
            mesh.visual = prepare.trimesh.visual.TextureVisuals(material=prepare.trimesh.visual.material.PBRMaterial(baseColorFactor=color))
            transform = np.eye(4); transform[0, 3] = index * 3
            scene.add_geometry(mesh, transform=transform)
        visual = ET.fromstring('<visual><origin xyz="0 2 0"/><geometry><mesh filename="package://description/a.dae" scale="2 2 2"/></geometry></visual>')
        with patch.object(prepare.trimesh, "load", return_value=scene):
            regions = prepare.visual_geometry(visual, ET.fromstring('<robot/>'), Path('/tmp/package'), 'description')
        self.assertEqual(len(regions), 2)
        np.testing.assert_allclose(regions[0][1], np.array([125, 173, 204, 255])/255)
        np.testing.assert_allclose(regions[1][1], np.array([8, 8, 8, 255])/255)
        np.testing.assert_allclose(regions[0][0].centroid, [0, 2, 0], atol=1e-12)
        np.testing.assert_allclose(regions[1][0].centroid, [6, 2, 0], atol=1e-12)
        self.assertEqual(sum(len(r[0].faces) for r in regions), 24)

    def test_named_urdf_material_overrides_mesh_material(self):
        robot = ET.fromstring('<robot><material name="blue"><color rgba="0.1 0.2 0.8 1"/></material></robot>')
        visual = ET.fromstring('<visual><geometry><box size="1 1 1"/></geometry><material name="blue"/></visual>')
        regions = prepare.visual_geometry(visual, robot, Path('/tmp/package'), 'description')
        self.assertEqual(regions[0][1], [0.1, 0.2, 0.8, 1])

    def test_unsupported_texture_is_not_silently_discarded(self):
        visual = ET.fromstring('<visual><geometry><box size="1 1 1"/></geometry><material><texture filename="paint.png"/></material></visual>')
        with self.assertRaisesRegex(ValueError, "texture"):
            prepare.visual_geometry(visual, ET.fromstring('<robot/>'), Path('/tmp/package'), 'description')

    def test_photo_finish_clips_paint_without_changing_the_surface(self):
        mesh = prepare.smooth_stl(prepare.trimesh.creation.box())
        profile = {"rings": {"test": [{"axis": 2, "center": [0, 0, 0], "axial": [0, 0.25], "radius": [0, 1]}]},
                   "flange": {"link": "test", "axis": 2, "minimum": 0.25}}
        for name, color in [("white", [1, 1, 1, 1]), ("red", [1, 0, 0, 1]), ("silver", [0.7, 0.7, 0.7, 1])]:
            profile[name] = {"color": color, "metallic": 0, "roughness": 0.4}
        regions = prepare.paint_reference_regions("test", [(mesh, None, 0, 0)], profile)
        self.assertEqual(len(regions), 3)
        joined = prepare.trimesh.util.concatenate([region[0] for region in regions])
        self.assertGreater(len(joined.faces), len(mesh.faces))
        self.assertAlmostEqual(joined.area, mesh.area, places=12)
        np.testing.assert_allclose(joined.bounds, mesh.bounds)
        # Every new point remains on an original box face.
        np.testing.assert_allclose(np.max(np.abs(joined.vertices), axis=1), 0.5)

    def test_cylinder_normals_are_smooth_but_caps_keep_their_crease(self):
        source = prepare.trimesh.creation.cylinder(radius=1, height=2, sections=24)
        mesh = prepare.smooth_stl(source)
        corner_normals = mesh.vertex_normals[mesh.faces]
        side = np.abs(mesh.face_normals[:, 2]) < 0.01
        radial = mesh.triangles[side].copy(); radial[:, :, 2] = 0
        radial /= np.linalg.norm(radial, axis=2, keepdims=True)
        np.testing.assert_allclose(corner_normals[side], radial, atol=1e-12)
        cap = ~side
        np.testing.assert_allclose(corner_normals[cap], np.repeat(mesh.face_normals[cap, None, :], 3, axis=1), atol=1e-12)
        np.testing.assert_array_equal(mesh.triangles, source.triangles)

    def test_round_paint_boundary_cuts_across_a_coarse_triangle(self):
        mesh = prepare.trimesh.Trimesh([[-0.1, -0.1, 0], [0.1, -0.1, 0], [0, 0.2, 0]], [[0, 1, 2]],
                                      vertex_normals=[[0, 0, 1]]*3, process=False)
        material = {"color": [1, 1, 1, 1], "metallic": 0, "roughness": 0.4}
        profile = {"rings": {"test": [{"axis": 2, "center": [0, 0, 0], "axial": [-0.001, 0.001], "radius": [0.03, 0.05]}]},
                   "flange": {"link": "other"}, "white": material, "red": material, "silver": material}
        regions = prepare.paint_reference_regions("test", [(mesh, None, 0, 0)], profile)
        self.assertEqual(len(regions), 2)
        joined = prepare.trimesh.util.concatenate([r[0] for r in regions])
        self.assertAlmostEqual(joined.area, mesh.area, places=12)
        red = regions[1][0]
        self.assertAlmostEqual(red.area, np.pi*(0.05**2-0.03**2), delta=2e-6)
        radius = np.linalg.norm(red.vertices[:, :2], axis=1)
        self.assertGreater(np.count_nonzero(np.abs(radius-0.03)<2e-5), 100)
        self.assertGreater(np.count_nonzero(np.abs(radius-0.05)<2e-5), 100)
        np.testing.assert_allclose(joined.vertex_normals, np.tile([0, 0, 1], (len(joined.vertices), 1)), atol=1e-12)

    def test_paint_boundary_uses_the_actual_mesh_rim_polygon(self):
        from visual_finishes import mesh_ring_planes
        mesh = prepare.trimesh.creation.annulus(r_min=0.03, r_max=0.05, height=0.002, sections=12)
        ring = {"axis": 2, "center": [0, 0, 0], "axial": [-0.002, 0.002], "radius": [0.03, 0.05]}
        planes = mesh_ring_planes(mesh, ring)
        self.assertEqual(len(planes), 12)
        # A coarse source rim is a polygon, whose edges lie inside the ideal circle.
        np.testing.assert_allclose([offset for _, offset in planes], 0.03*np.cos(np.pi/12), atol=1e-8)
        points = mesh.vertices[np.abs(np.linalg.norm(mesh.vertices[:, :2], axis=1)-0.03)<1e-7]
        for normal, offset in planes:
            self.assertLessEqual(np.max(points @ normal-offset), 1e-8)
            self.assertGreaterEqual(np.count_nonzero(np.abs(points @ normal-offset)<1e-8), 4)

    def test_whole_head_finish_reaches_the_existing_part_boundary(self):
        mesh = prepare.smooth_stl(prepare.trimesh.creation.box())
        white = {"color": [1, 1, 1, 1], "metallic": 0, "roughness": 0.4}
        silver = {"color": [0.7, 0.7, 0.7, 1], "metallic": 0.85, "roughness": 0.28}
        profile = {"rings": {}, "flange": {"link": "head", "wholeLink": True},
                   "white": white, "red": white, "silver": silver}
        regions = prepare.paint_reference_regions("head", [(mesh, None, 0, 0)], profile)
        self.assertEqual(len(regions), 1)
        self.assertEqual(regions[0][1], silver['color'])
        np.testing.assert_array_equal(regions[0][0].triangles, mesh.triangles)

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
