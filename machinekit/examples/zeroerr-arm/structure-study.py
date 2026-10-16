#!/usr/bin/env python3
"""Conservative zero-speed force/moment bounds and ideal tube/bolt-group screens.

This is not a joint-compliance, fatigue, preload or repeatability calculation.
All downstream masses are retained in triangle-inequality bounds, even where
they would be upstream of a particular tube section: that deliberately overbounds.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("arm_engineering", ROOT / "engineering-study.py")
engineering = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engineering)


def cantilever(force, moment, length, youngs, inertia):
    return (force*length**3/(3*youngs*inertia) + moment*length**2/(2*youngs*inertia),
            force*length**2/(2*youngs*inertia) + moment*length/(youngs*inertia))


def main():
    current = json.loads((ROOT / "build/straight/current.json").read_text())
    layout_path = Path(current["layoutPath"])
    layout = json.loads(layout_path.read_text())
    design = layout["straightForearm"]
    config = design["config"]
    audit = design["audit"]
    directory = ROOT / "build/straight"
    raw = (directory / "engineering-input.json").read_bytes()
    geometry = json.loads(raw)
    if geometry["layoutSha256"] != hashlib.sha256(layout_path.read_bytes()).hexdigest():
        raise ValueError("Stale straight engineering geometry")
    sizing = json.loads((directory / "sizing-report.json").read_text())
    if sizing["geometrySha256"] != hashlib.sha256(raw).hexdigest():
        raise ValueError("Stale sizing report")
    study_config = json.loads((directory / "engineering-config.json").read_text())
    if sizing["configSha256"] != hashlib.sha256((directory / "engineering-config.json").read_bytes()).hexdigest():
        raise ValueError("Changed engineering assumptions")
    override_path = ROOT / "straight-engineering-overrides.json"
    overrides = json.loads(override_path.read_text())
    if study_config["straightVariant"]["overrideSha256"] != hashlib.sha256(override_path.read_bytes()).hexdigest():
        raise ValueError("Changed structural assumptions; rerun load study")
    study = engineering.Study(geometry, study_config)
    downstream = {"forearm"}
    for _ in geometry["joints"]:
        downstream.update(j["child"] for j in geometry["joints"] if j["parent"] in downstream)
    bodies = [b for b in study.bodies if b[0] in downstream or b[0] == "tool+payload" or b[0].startswith("cableBracket4Distal")]
    if not bodies or not any(b[0] == "tool+payload" for b in bodies):
        raise ValueError("Missing downstream loads")
    app = study_config["application"]
    lo, hi = np.deg2rad(app["syntheticJointRangeDeg"])
    poses = [np.zeros(6), np.array([.2, -.8, 1., .1, -.6, .2])]
    poses += [np.eye(6)[i]*angle for i in range(6) for angle in (lo, hi)]
    poses += [np.array([lo+(hi-lo)*engineering.halton(n,b) for b in (2,3,5,7,11,13)])
              for n in range(1,app["sampleCount"]+1)]
    forearm = next(o for o in layout["occurrences"] if o["id"] == "forearm")
    matrix = np.array(forearm["matrix"]).reshape(3,4)
    sections = [(matrix[:,3]+matrix[:,:3] @ [0,y,0])/1000
                for y in (0,audit["nearShoulderY"],audit["farShoulderY"])]
    forearm_active = next(b[4] for b in bodies if b[0] == "forearm")
    alpha = np.deg2rad(app["jointAccelerationDegS2"])
    peak_force, peak_moments, witness = 0., np.zeros(3), [None]*3
    mass_factor = app["massSensitivityFactor"]
    for q in poses:
        rotations = [engineering.rotation(a,v) for a,v in zip(study.axes,q)]
        def transform(point, active):
            p, r = point.copy(), np.eye(3)
            for i in reversed(active):
                p = study.origins[i] + rotations[i] @ (p-study.origins[i])
                r = rotations[i] @ r
            return p,r
        axes, origins = [], []
        for i in range(6):
            point,r = transform(study.origins[i],range(i))
            origins.append(point); axes.append(r @ study.axes[i])
        current_sections = [transform(p,forearm_active)[0] for p in sections]
        total_force, moments = 0., np.zeros(3)
        for name,mass,point,inertia,active in bodies:
            factor = 1 if name == "tool+payload" else mass_factor
            point,_ = transform(point,active)
            acceleration = sum(np.linalg.norm(np.cross(axes[i],point-origins[i]))*alpha for i in active)
            force = mass*factor*(engineering.G+acceleration)
            couple = np.linalg.eigvalsh(inertia).max()*factor*len(active)*alpha
            total_force += force
            moments += [np.linalg.norm(point-section)*force+couple for section in current_sections]
        # Reserve the load study's diagonal reflected-inertia approximation.
        moments += study.reflected[3:].sum()*alpha
        total_force *= app["designFactor"]
        moments *= app["designFactor"]
        peak_force = max(peak_force,total_force)
        for i in range(3):
            if moments[i] >= peak_moments[i]:
                peak_moments[i] = moments[i]; witness[i] = q.tolist()
    elastic = overrides["elasticScreen"]
    radius = config["tubeOuterDiameterMm"]/2000
    inner = radius-config["tubeWallMm"]/1000
    area = np.pi*(radius**2-inner**2)
    inertia = np.pi*(radius**4-inner**4)/4
    polar = 2*inertia
    length = audit["tubeLengthMm"]/1000
    moment = float(max(peak_moments))
    youngs, shear = elastic["youngsModulusGPa"]*1e9, elastic["shearModulusGPa"]*1e9
    nominal_normal = peak_force/area+moment*radius/inertia
    nominal_shear = moment*radius/polar
    concentrated = np.sqrt(nominal_normal**2+3*nominal_shear**2)*elastic["holeStressConcentrationFactor"]
    allowable = elastic["assumedYieldMPa"]*1e6/elastic["yieldDesignFactor"]
    screws = config["fastening"]["screwsPerEnd"]
    pilot_radius = audit["spigotDiameterMm"]/2000
    # Bending distributes over a circular group with sum(y_i^2)=n*r^2/2.
    per_screw = peak_force/screws + 2*moment/(screws*pilot_radius)
    bolt_shear_capacity = elastic["nominalM5StressAreaMm2"]*elastic["assumedBoltShearAllowableMPa"]
    bearing_wall = config["tubeWallMm"]-config["fastening"]["spotfaceDepthMm"]
    bearing_capacity = bearing_wall*5*elastic["assumedTubeBearingAllowableMPa"]
    deflection, angle = cantilever(peak_force,moment,length,youngs,inertia)
    np.testing.assert_allclose(cantilever(1,0,1,1,1),[1/3,1/2],atol=1e-14)
    np.testing.assert_allclose(cantilever(0,1,1,1,1),[1/2,1],atol=1e-14)
    np.testing.assert_allclose(6*engineering.G*.15,8.825985,atol=1e-12)
    baseline = json.loads((ROOT / "build/revised/sizing-report.json").read_text())
    if baseline["configSha256"] != hashlib.sha256((ROOT/"revised-engineering-config.json").read_bytes()).hexdigest() or baseline["layoutSha256"] != hashlib.sha256((ROOT/"revised-layout.json").read_bytes()).hexdigest():
        raise ValueError("Stale tapered comparison report")
    issues = []
    if concentrated > allowable: issues.append("Tube bulk stress screen exceeds assumed allowable")
    if per_screw > bolt_shear_capacity: issues.append("Bolt-group shear screen exceeds assumed allowable")
    if per_screw > bearing_capacity: issues.append("Tube bearing screen exceeds assumed allowable")
    result = {"status":"prototype_changes_required" if issues else "preliminary_screens_clear_hardware_unqualified",
              "issues":issues,"layoutSha256":geometry["layoutSha256"],
              "geometrySha256":hashlib.sha256(raw).hexdigest(),"samplePoses":len(poses),
              "analysisScriptSha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "assumptionSha256":hashlib.sha256(override_path.read_bytes()).hexdigest(),
              "downstreamMembers":[b[0] for b in bodies],"loadBasis":"Triangle-inequality gravity + zero-speed acceleration bounds, heavier structural mass and design factor. Includes upstream mass at tube sections conservatively; no velocity coupling or fatigue.",
              "forceBoundN":peak_force,"sectionMomentBoundsNm":dict(zip(["supplierMount","tubeNear","tubeFar"],peak_moments.tolist())),
              "momentWitnessRad":witness,"idealTube":{"areaMm2":area*1e6,"bendingInertiaMm4":inertia*1e12,
              "polarInertiaMm4":polar*1e12,"bendingRigidityNm2":youngs*inertia,"torsionalRigidityNm2":shear*polar,
              "concentratedStressBoundMPa":concentrated/1e6,"assumedAllowableMPa":allowable/1e6,
              "stressUtilization":concentrated/allowable,"idealCantileverDeflectionMmUnderLoadBounds":deflection*1000,
              "idealCantileverSlopeRadUnderLoadBounds":angle,"idealTorsionRadUnderLoadBounds":moment*length/(shear*polar)},
              "connection":{"screwsPerEnd":screws,"perScrewLoadBoundN":per_screw,
              "assumedBoltShearCapacityN":bolt_shear_capacity,"boltShearUtilization":per_screw/bolt_shear_capacity,
              "minimumTubeWallAtSpotfaceMm":bearing_wall,"assumedTubeBearingCapacityN":bearing_capacity,
              "tubeBearingUtilization":per_screw/bearing_capacity,"preloadSpecified":False,"dowelLoadShareCredited":False},
              "mass":{"straightArmKg":sizing["nominalArmMassKg"],"taperedArmKg":baseline["nominalArmMassKg"],
              "deltaKg":sizing["nominalArmMassKg"]-baseline["nominalArmMassKg"]},
              "limitations":["Ideal unperforated tube stiffness; local holes and adapter/bolt joint compliance are not included.",
              "The conservative loading applies the same moment bound to bending and torsion; it is not an achievable single load case.",
              "Linear elastic deflection is not whole-arm stiffness, TCP accuracy or repeatability.",
              "Thread stripping, preload, fit tolerances, slip, fatigue, cable drag and manufacturing are unqualified.",
              "Shoulder drive remains below the full-workspace load requirement."]}
    output = directory / "structure-report.json"
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"forceBoundN":peak_force,"momentBoundNm":moment,"stressUtilization":concentrated/allowable,
                      "tubeBearingUtilization":per_screw/bearing_capacity,"massDeltaKg":result["mass"]["deltaKg"]},indent=2))
    print(output)


if __name__ == "__main__":
    main()
