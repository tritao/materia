"""Solve and compare bonded-frame compliance meshes with explicit idealized joints/supports."""

import argparse
import importlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess

from frame_mesh import GMSH_VERSION
from mesh_cache import cached_mesh
from frame_solver import assess, read_results, read_energies, run_solver, write_deck
from packet import SOLVER_UNITS, digest, elastic_input, finite, validate
from solver_fixtures import CCX_VERSION
from tet_fixture import verify
from spring_fixture import verify as verify_springs
from compliance_budget import budget, tool_case_packet
from provenance import capture_sources
from leveling import add_influence_steps, offsets_from_tables
from leveling_fixture import verify as verify_leveling


def validate_config(config):
    finite(config)
    if config["schemaVersion"] != 1 or config["model"] != "upper-bound-bonded-frame-rigid-axes":
        raise ValueError("Unsupported frame study configuration")
    for name in ("mineral-cast", "machined-steel"):
        material = config["materials"][name]
        elastic_input(material["youngsPa"], material["poissonRatio"], material["densityKgPerM3"])
        if material["assumed"] is not True or not material["source"]:
            raise ValueError("This initial study requires explicit property assumptions and provenance")
    if set(config["supportConfigurations"]) != {"soft-three-point", "stiff-four-springs"}:
        raise ValueError("Both support brackets are required")
    for support in config["supportConfigurations"].values():
        if support["verticalSpringNPerMm"] < 0 or not support["description"]:
            raise ValueError("Invalid support configuration")
    for key in ("cuttingForceN", "footContactRadiusMm", "payloadKg", "gravityMPerS2"):
        if not isinstance(config[key], (int, float)) or isinstance(config[key], bool) or config[key] <= 0:
            raise ValueError(f"Invalid positive study input {key}")
    for key in ("equilibriumRelativeTolerance", "meshRelativeTolerance"):
        if not 0 < config[key] <= .05:
            raise ValueError(f"Invalid numerical acceptance tolerance {key}")
    if not config["limitations"]:
        raise ValueError("Model limitations must be stated")


def convergence(results):
    results = [r for r in results if r["throughThickness"]["allSectionsResolved"]]
    if len(results) < 2:
        return {"available":False,"worstRelativeChange":None}
    changes = {}
    for name in results[-1]["supports"]:
        a,b = (r["supports"][name]["solution"]["complianceUmPerN"] for r in results[-2:])
        changes[name] = max(math.sqrt(sum((a[i][j]-b[i][j])**2 for i in range(3)))/math.sqrt(sum(b[i][j]**2 for i in range(3))) for j in range(3))
    return {"available":True,"worstRelativeChange":max(changes.values()),"supportRelativeChanges":changes}


def markdown(report):
    lines = ["# Upper-bound bonded-frame static study", "", report["status"], "",
             "Bonded inserts and clamp pads, rigid moving axes. Kinematic guide/drive patch equations stiffen the rail/seat regions and are acceptable only for this upper-bound model.",
             "Gravity is a separate reaction case. The compliance matrix uses independent XYZ unit load pairs without gravity subtraction.", "",
             f"Packet SHA256: `{report['manifestSha256']}`", f"Configuration SHA256: `{report['configurationSha256']}`", "",
             "| Target mm | Support bound | Cxx µm/N | Cyy µm/N | Czz µm/N | Worst ±500 N box µm |", "| --- | --- | --- | --- | --- | --- |"]
    for mesh in report["meshes"]:
        for name,result in mesh["supports"].items():
            solution=result["solution"]; c=solution["complianceUmPerN"]
            lines.append(f"| {mesh['sizeMm']:g} | {name} | {c[0][0]:.6f} | {c[1][1]:.6f} | {c[2][2]:.6f} | {solution['worstLoadBoxCorner']['normUm']:.4f} |")
    fine=report["meshes"][-1]
    for name,result in fine["supports"].items():
        solution=result["solution"]
        lines += ["",f"## {name}","", "Compliance matrix (µm/N):", "", "```json",json.dumps(solution["complianceUmPerN"]),"```", "",
                  "Principal compliances and world directions:", ""]
        for mode in solution["principalCompliances"]:
            lines.append(f"- {mode['complianceUmPerN']:.6f} µm/N: {mode['worldDirection']}")
        lines += ["", "| Unit pair | Base casting share | Column casting share | Inserts share | Clamp pads share | Support share |", "| --- | --- | --- | --- | --- | --- |"]
        for case in solution["cases"]:
            if case["case"] in ("x","y","z") and "complianceShares" in case:
                share=case["complianceShares"]
                lines.append("| "+case["case"]+" | "+" | ".join(f"{100*share.get(k,0):.2f}%" for k in ("BASE_CASTING","COLUMN_CASTING","INSERTS","CLAMP_PADS","SUPPORT_SPRINGS"))+" |")
    lines += ["", "## Mesh and numerical gates", "", "| Target mm | Quadratic tets | All thickness probes pass | Minimum elements crossed |", "| --- | --- | --- | --- |"]
    for mesh in report["meshes"]:
        resolution=mesh["throughThickness"]
        lines.append(f"| {mesh['sizeMm']:g} | {mesh['elements']} | {resolution['allSectionsResolved']} | {min(p['quadraticElementsCrossed'] for p in resolution['probes'])} |")
    lines += ["", "Convergence: "+json.dumps(report["convergence"]), "", "Strain-energy cross-check: "+json.dumps(fine.get("energyCrossCheck")), ""]
    for name,result in fine["supports"].items():
        gravity=next(c for c in result["solution"]["cases"] if c["case"]=="gravity")
        lines += ["", f"Gravity reactions, {name} (N):", "", "| Foot | X | Y | Z |", "| --- | --- | --- | --- |"]
        for station,force in gravity["supportReactionsN"].items():
            lines.append("| "+station+" | "+" | ".join(f"{v:.4f}" for v in force)+" |")
    lines += ["", "## Compliance budget", "", report["geometryDecision"], "",
              f"Provisional casting share target: {100*report['configuration']['frameShareTarget']['maximumFraction']:g}%. Catalogue proxies and unmeasured contact springs remain conditional inputs.", ""]
    for name,result in fine["supports"].items():
        budget_result=result["complianceBudget"]
        lines += [f"### {name}", "", "| Element | X share | Y share | Z share |", "| --- | --- | --- | --- |"]
        for group in budget_result["componentComplianceUmPerN"]:
            lines.append("| "+group+" | "+" | ".join(f"{100*budget_result['directionalShares'][a][group]:.2f}%" for a in "xyz")+" |")
        lines += ["", "Casting shares of total loop: "+", ".join(f"{a.upper()} {100*v:.2f}%" for a,v in budget_result["castingLoopShares"].items()), ""]
    lines += ["", "Support difference: "+json.dumps(report["supportDifference"]), ""]
    lines += ["", "## Qualification", "", "Every counted level must pass the reported through-thickness checks. The last refinement must be within 5% in all unit-response vectors. The base-casting 1% stiffening check must agree with the energy derivative within 2.5%. Catalogue proxies and clamp/taper assumptions do not establish physical machine accuracy.", ""]
    lines += ["- "+item for item in report["configuration"]["limitations"]]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("frame-study.json"))
    parser.add_argument("--ccx", default="ccx")
    parser.add_argument("--budget",type=Path,default=Path(__file__).with_name("compliance-budget.json"))
    parser.add_argument("--tool-case", help="Named compliance-budget tool case; moves both FE contact references")
    parser.add_argument("--sizes", nargs="+", type=float, default=[40, 30])
    parser.add_argument("--resume",action="store_true",help="Reuse completed solver outputs only when regenerated input bytes match")
    parser.add_argument("--threads",type=int,choices=[1,2,4],default=1)
    parser.add_argument("--timeout", type=int, default=1800, help="Per-mesh solver timeout in seconds")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "build/fem/static-study")
    args = parser.parse_args()
    report_path = args.output / "report.json"
    for name in ("report.json", "report.md"):
        (args.output / name).unlink(missing_ok=True)
    gmsh = None
    try:
        original_manifest=args.manifest.resolve()
        input_digest=digest(original_manifest)
        snapshot=args.output.resolve()/f"packet-{input_digest[:16]}"
        if not snapshot.exists():
            shutil.copytree(original_manifest.parent,snapshot)
        args.manifest=snapshot/"manifest.json"
        if digest(args.manifest)!=input_digest:
            raise ValueError("Input packet changed while being frozen")
        validated = validate(args.manifest)
        packet = json.loads(args.manifest.read_text())
        config = json.loads(args.config.read_text())
        if args.tool_case:
            packet = tool_case_packet(packet,json.loads(args.budget.read_text()),args.tool_case)
        validate_config(config)
        source_names=("static_study.py","leveling.py","leveling_fixture.py","mesh_cache.py","frame_mesh.py","frame_solver.py","packet.py","tet_fixture.py","spring_fixture.py","ccx_deck.py","mesh_resolution.py","compliance.py","compliance_budget.py","provenance.py","solver_fixtures.py")
        source_hashes={name:digest(Path(__file__).with_name(name)) for name in source_names}
        provenance_output=args.output.resolve()/"provenance"
        source_build=capture_sources(Path(__file__).resolve().parents[4],provenance_output)
        configuration_hash=digest(args.config)
        packet_hash=digest(args.manifest)
        budget_hash=digest(args.budget)
        if any("parentId" not in p for p in packet["parts"]) or any("railWidthMm" not in g for g in packet["guides"]):
            raise ValueError("Re-export the packet: static study requires parent links and guide footprints")
        if any(not math.isfinite(s) or s <= 0 for s in args.sizes) or any(a <= b for a, b in zip(args.sizes, args.sizes[1:])):
            raise ValueError("Mesh sizes must be positive and strictly decrease")
        executable = shutil.which(args.ccx)
        if executable is None:
            raise ValueError(f"CalculiX {CCX_VERSION} is required; pass --ccx")
        ccx = Path(executable).resolve()
        version = subprocess.run([str(ccx), "-v"], capture_output=True, text=True, timeout=10)
        if version.returncode not in (0, 201) or not re.search(rf"\bVersion\s+{re.escape(CCX_VERSION)}\b", version.stdout):
            raise ValueError(f"Expected CalculiX {CCX_VERSION}: {version.stdout + version.stderr}")
        gmsh = importlib.import_module("gmsh")
        if gmsh.__version__ != GMSH_VERSION:
            raise ValueError(f"Expected Gmsh {GMSH_VERSION}")
        fixture = verify(gmsh, ccx, args.output / "tet-verification")
        spring_fixture = verify_springs(ccx,args.output/"spring-verification")
        leveling_fixture = verify_leveling(ccx,args.output/"leveling-verification")
        def solve_case(output, write):
            previous_input=digest(output/"frame.inp") if (output/"frame.inp").exists() else None
            previous_result=(output/"frame.dat").exists() and (output/"solver.log").exists() and "Job finished" in (output/"solver.log").read_text() and "*ERROR" not in (output/"solver.log").read_text().upper() and re.search(rf"Using up to\s+{args.threads}\s+cpu",(output/"solver.log").read_text()) is not None
            model=write()
            if not (args.resume and previous_result and previous_input==model["inputSha256"]):
                run_solver(ccx,output,args.timeout,args.threads)
            else:
                print(f"Reusing identical completed deck: {output}",flush=True)
            return model
        results = []
        for size in args.sizes:
            output = (args.output / f"mesh-{size:g}").resolve()
            output.mkdir(parents=True, exist_ok=True)
            print(f"Meshing bonded frame at {size:g} mm", flush=True)
            gmsh.initialize()
            gmsh.logger.start()
            try:
                mesh = cached_mesh(gmsh, packet, args.manifest, config, size, output)
            finally:
                gmsh_messages=gmsh.logger.get()
                (output/"gmsh.log").write_text("\n".join(gmsh_messages)+"\n")
                gmsh.logger.stop()
                gmsh.finalize()
            if not mesh["throughThickness"]["allSectionsResolved"]:
                (output/"thickness-failure.json").write_text(json.dumps(mesh["throughThickness"],indent=2)+"\n")
                raise ValueError("Mesh fails through-thickness gate; see thickness-failure.json")
            supports = {}
            for support_name in config["supportConfigurations"]:
                support_output = output / support_name
                support_output.mkdir(parents=True,exist_ok=True)
                print(f"Solving {support_name}; log: {support_output / 'solver.log'}",flush=True)
                offsets = {}
                stiffness = config["supportConfigurations"][support_name]["verticalSpringNPerMm"]
                if stiffness:
                    calibration = support_output / "leveling-influence"
                    calibration.mkdir(exist_ok=True)
                    def write_leveling():
                        model = write_deck(mesh,packet,config,calibration,support_name,gravity_only=True)
                        return add_influence_steps(calibration/"frame.inp",model)
                    level_model = solve_case(calibration,write_leveling)
                    level_tables = read_results((calibration/"frame.dat").read_text(),5)
                    offsets, leveling_check = offsets_from_tables(level_model,level_tables)
                model = solve_case(support_output,lambda:write_deck(mesh,packet,config,support_output,support_name,ground_offsets=offsets))
                text=(support_output/"frame.dat").read_text()
                tables=read_results(text,len(model["cases"]))
                energy=read_energies(text,len(model["cases"]),model["energyGroups"])
                solution=assess(packet,config,model,tables,energy)
                supports[support_name]={"levelingCalibration":leveling_check if stiffness else None,"model":model,"solution":solution,"resultSha256":digest(support_output/"frame.dat")}
            result = {"sizeMm": size, "nodes": len(mesh["nodes"]), "elements": sum(len(v) for v in mesh["elements"].values()),
                      "meshSha256": mesh["meshSha256"], "supports": supports,
                      "gmshLibrarySha256": mesh["gmshLibrarySha256"], "minimumJacobianDeterminant": mesh["minimumJacobianDeterminant"],
                      "bonds": mesh["bonds"], "patches": mesh["patches"],"meshPartitions":mesh["partitions"],"throughThickness":mesh["throughThickness"],"gmshWarnings":mesh.get("cachedGmshWarnings",[line for line in gmsh_messages if "Warning" in line or "Error" in line])}
            if size == args.sizes[-1]:
                check_output=output/"base-stiffening-check"
                check_output.mkdir(exist_ok=True)
                check_model=solve_case(check_output,lambda:write_deck(mesh,packet,config,check_output,"soft-three-point",stiffen_group="BASE_CASTING",unit_axis="x"))
                check_text=(check_output/"frame.dat").read_text()
                check_tables=read_results(check_text,1)
                check_u=check_tables[1,"displacements","OBS"]
                refs=check_model["references"]
                checked_cxx=1000*(check_u[refs["head"]["translation"]][0]-check_u[refs["table"]["translation"]][0])
                baseline=supports["soft-three-point"]["solution"]
                errors={}
                for i,axis in enumerate("x"):
                    share=next(c["complianceShares"]["BASE_CASTING"] for c in baseline["cases"] if c["case"]==axis)
                    actual=1-checked_cxx/baseline["complianceUmPerN"][i][i]
                    predicted=.01*share
                    errors[axis]={"predictedReductionFraction":predicted,"actualReductionFraction":actual,"relativeError":abs(actual/predicted-1)}
                if max(v["relativeError"] for v in errors.values())>.025:
                    raise ValueError(f"Group stiffening/energy cross-check failed: {errors}")
                result["energyCrossCheck"]={"group":"BASE_CASTING","modulusMultiplier":1.01,"directions":errors,"passed":True}
            (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            results.append(result)
            del mesh, tables
        budget_inputs=json.loads(args.budget.read_text())
        for result in results:
            for support in result["supports"].values():
                support["complianceBudget"]=budget(packet,config,budget_inputs,support["solution"])
        fine=results[-1]["supports"]
        soft=fine["soft-three-point"]["solution"]["worstLoadBoxCorner"]["normUm"]
        stiff=fine["stiff-four-springs"]["solution"]["worstLoadBoxCorner"]["normUm"]
        support_difference={"softWorstLoadBoxUm":soft,"stiffWorstLoadBoxUm":stiff,"changeRelativeToStiff":(soft-stiff)/stiff}
        eligible=[s["complianceBudget"]["geometryEligible"] for s in fine.values()]
        decision=("Context tool case; no geometry decision" if all(v is None for v in eligible) else
                  "Casting share reaches target; assess stiffness candidates" if any(eligible) else
                  "Casting share below target for this decision case; lightening requires modal and travel screening")
        if source_hashes!={name:digest(Path(__file__).with_name(name)) for name in source_names} or configuration_hash!=digest(args.config) or packet_hash!=digest(args.manifest) or budget_hash!=digest(args.budget):
            raise ValueError("Analysis sources or inputs changed during solve; rerun with --resume to verify identical decks")
        change = convergence(results)
        converged = results[-1]["throughThickness"]["allSectionsResolved"] and change["available"] and change["worstRelativeChange"] <= config["meshRelativeTolerance"]
        report = {"status": "Frame compliance solve passed numerical checks" if converged else "Frame solve completed; mesh convergence NOT established",
                  "sourceManifestPath":str(original_manifest),"frozenManifestPath":str(args.manifest),"geometryDecision":decision,"supportDifference":support_difference,"analysisSources":source_build,"geometrySourceBuild":packet["provenance"]["sourceBuild"],"budgetConfigurationSha256":digest(args.budget),
                  "converged": converged, "units": SOLVER_UNITS, "responseUnits": "micrometres",
                  "packetDensityMassChecks":validated["densityMassChecks"],"manifestSha256": validated["manifestSha256"], "configurationSha256": digest(args.config), "configuration": config,
                  "solverThreads":args.threads,"solverVersion": version.stdout.strip(), "solverSha256": digest(ccx),
                  "tetVerification": fixture,"supportSpringVerification":spring_fixture,"levelingInfluenceVerification":leveling_fixture,
                  "sourceSha256":source_hashes,
                  "toolCase":packet.get("analysisToolCase"),"analysisContactMm":packet["datums"]["toolTipMm"],"jointsMm": packet["jointsMm"], "convergence": change, "meshes": results}
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        (args.output / "report.md").write_text(markdown(report))
        print(f"{report['status']}; report: {report_path}")
        return 0 if converged else 2
    except Exception as error:
        parser.exit(1, f"Static frame study failed: {error}\n")
    finally:
        if gmsh is not None and gmsh.isInitialized():
            gmsh.finalize()


if __name__ == "__main__":
    raise SystemExit(main())
