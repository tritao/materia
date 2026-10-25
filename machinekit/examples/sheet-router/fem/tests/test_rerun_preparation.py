"""Input corruption, mechanics invariants and no-solve reserve controls."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

FEM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(FEM))
import executed_rerun as executed
import mount_compliance as mount
import mount_clearance_plan as clearance


class PreparationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=FEM.parents[3]
        cls.native=mount.witness(root/'build/sheet-router-evidence/mount-geometry-20261023.tar.gz',
            FEM.parent/'mounting/geometry-validation.json')

    def control(self):
        clock=dict(checks=[dict(kind=k,status=1) for k in (1,2,3)],jointMappedBounds=[[0,0,0]]*3,mappedAccelerationStepBound=0.)
        metadata=dict(control=dict(feedOverride=1.),scope='Observed actual host executor',clockCertificate=clock,
            runnerReport=dict(completionChecked=True),config=dict(jerkX=50.,jerkY=50.,jerkZ=50.,
                driveJerkX=100.,driveJerkY=100.,driveJerkZ=100.,clockAdmission=dict(accelerationStepMPerS2=1e-5)))
        return metadata,dict(clockCertificate=copy.deepcopy(clock))

    def test_clock_accepts_declared_drive_reserve(self):
        meta,manifest=self.control();meta['clockCertificate']['jointMappedBounds'][1]=[.2,2.,75.]
        manifest['clockCertificate']=copy.deepcopy(meta['clockCertificate'])
        executed.check_clock(meta,manifest)

    def test_offline_control_is_not_executed(self):
        meta,manifest=self.control();meta['scope']='Offline ordinary compiler prefix/splice/tail'
        with self.assertRaises(ValueError):executed.check_clock(meta,manifest)

    def test_above_one_clock_is_rejected(self):
        meta,manifest=self.control();meta['control']['feedOverride']=1.5
        with self.assertRaises(ValueError):executed.check_clock(meta,manifest)

    def test_missing_jerk_cannot_hide_behind_extra_certificate(self):
        meta,manifest=self.control();meta['clockCertificate']['checks']=[dict(kind=k,status=1) for k in (1,2,4)]
        manifest['clockCertificate']=copy.deepcopy(meta['clockCertificate'])
        with self.assertRaises(ValueError):executed.check_clock(meta,manifest)

    def test_nan_cap_and_seam_fail(self):
        for field in ('cap','seam'):
            meta,manifest=self.control()
            if field=='cap':meta['config']['driveJerkY']=float('nan')
            else:meta['clockCertificate']['mappedAccelerationStepBound']=float('nan')
            manifest['clockCertificate']=copy.deepcopy(meta['clockCertificate'])
            with self.assertRaises(ValueError):executed.check_clock(meta,manifest)

    def test_excessive_drive_jerk_fails(self):
        meta,manifest=self.control();meta['clockCertificate']['jointMappedBounds'][0]=[.2,2.,101.]
        manifest['clockCertificate']=copy.deepcopy(meta['clockCertificate'])
        with self.assertRaises(ValueError):executed.check_clock(meta,manifest)

    def test_grid_pose_binding(self):
        meta=dict(fixture=dict(placement=dict(originM=[1.6,3.15,.2])))
        bed,pose=executed.pose_for('y3150/p8/exports/3150-nominal',meta)
        self.assertEqual(bed,3150);self.assertEqual(pose,dict(x=1600.,y=3150.,z=200.))
        meta['fixture']['placement']['originM'][0]=.8
        with self.assertRaises(ValueError):executed.pose_for('y3150/p8/exports/3150-nominal',meta)

    def test_index_cannot_escape_dataset(self):
        with self.assertRaises(ValueError):executed.inside(Path('/tmp/dataset'),'../../other')
        with self.assertRaises(ValueError):executed.inside(Path('/tmp/dataset'),'/etc/passwd')

    def test_no_modal_replay_below_reserve(self):
        with patch.object(executed.shutil,'disk_usage',return_value=SimpleNamespace(free=5_000_000_000)):
            with self.assertRaises(RuntimeError):executed.reserve()
            with self.assertRaises(RuntimeError):executed.replay({},None,None,None,None)

    def test_reaction_weights_conserve_force_and_moment(self):
        for point in (-250.,-125.,0.,62.5,250.):
            pair,weights=mount.weights([-125.,0.,125.],point)
            self.assertAlmostEqual(float(sum(weights)),1.)
            self.assertAlmostEqual(float(weights@np.array(pair)),point)
        _,weights=mount.weights([-125.,0.,125.],250.)
        self.assertAlmostEqual(float(weights@weights),5.)

    def test_section_tensor_dimensions(self):
        area,centre,I=mount.section([(1.,2.,4.,0.,0.)])
        self.assertEqual(area,8.);np.testing.assert_allclose(centre,[0.,0.])
        np.testing.assert_allclose(I,np.diag([4*2**3/12,2*4**3/12]))
        with self.assertRaises(ValueError):mount.section([(-1.,2.,4.,0.,0.)])

    def test_native_geometry_load_paths_and_si_projection(self):
        actual=mount.screen(self.native);proposal=mount.screen(self.native,True)
        self.assertGreater(proposal['worst']['equivalentTangentialStiffnessNPerM'],2*actual['worst']['equivalentTangentialStiffnessNPerM'])
        self.assertAlmostEqual(actual['geometry']['rackBoltGripMm'],13.6)
        for result in (actual,proposal):
            worst=result['worst'];C=np.array(worst['flexibilityMatrixMPerN']);load=np.array(worst['loadDirection'])
            np.testing.assert_allclose(C,C.T,rtol=0,atol=1e-20)
            self.assertGreater(float(np.linalg.eigvalsh(C).min()),0.)
            self.assertAlmostEqual(float(load@C@load),1/worst['equivalentTangentialStiffnessNPerM'],places=18)
            self.assertAlmostEqual(sum(worst['projectedTermMPerN'].values()),float(load@C@load),places=18)

    def test_proposed_anchor_mass_uses_real_hardware(self):
        proposal=mount.screen(self.native,True)
        parts={p['id']:p for p in self.native['assembly/mass.json']['parts']}
        expected=2*sum(parts[k]['massKg'] for k in ('anchor-bolt1','anchor-nut1','anchor-washer1'))
        self.assertAlmostEqual(proposal['additionalHardwareMassKg'],expected)

    def test_recipe_identity_and_material_are_preserved(self):
        records=mount.parameters(self.native['assembly/assembly.json'])
        self.assertEqual(records['receiver']['values']['stock'],'C45')
        self.assertEqual(records['pad']['values']['stock'],'weldable-mild-steel')
        self.assertEqual(records['rack']['values']['designation'],'29-15-055')

    def test_explicit_mesh_frame_rotation(self):
        q=dict(qx=0.,qy=0.,qz=0.,qw=1.)
        np.testing.assert_allclose(clearance.rotate(q,[0,0,1]),[0,0,1])
        q['qw']=2.
        with self.assertRaises(ValueError):clearance.rotate(q,[0,0,1])


if __name__=='__main__':unittest.main()
