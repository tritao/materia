package machinekit.tooling;

import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.process.FillAnalysis;

/** Neutral process results enter the same fabrication manifest/readiness gate. */
class ToolingProcessExport {
	public static function write(p:CastingMoldPlan, spec:CastingProcessSpec, gravityMps2:Float, dir:String,
			review:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview>, ?fabrication:materia.tooling.export.FabricationSpec,
			?foundation:machinekit.structural.MineralFoundation):Array<ToolingValidation> {
		var report = CastingProcessAnalysis.analyze(p, spec, gravityMps2, fabrication);
		var checks = report.validation.filter(v -> StringTools.startsWith(v.rule, "process."));
		if (review == null || foundation == null)
			checks.push(FillAnalysis.check("process.fill.cad-volume-agreement", [p.document.input.id], ToolingCheckStatus.Pending,
				"Records-only fill geometry has not been cross-checked against reconstructed/source CAD.", []));
		else {
			if (review.fingerprint != report.sourceFingerprint)
				throw "Stale process CAD review";
			var actualInput = FoundationToolingAdapter.snapshot(foundation, p.document.input.id, p.document.input.revision);
			if (ToolingCodec.inputFingerprint(actualInput) != ToolingCodec.inputFingerprint(p.document.input))
				throw "Process CAD foundation differs from source revision";
			var resources:Array<cadkit.modeling.Part> = [];
			var difference = 0.0;
			try {
				var local = foundation.body.geometry();
				resources.push(local);
				var actual = machinekit.assembly.PosedParts.moved(local, p.document.setup.sourceToMold);
				resources.push(actual);
				if (Math.abs(report.fill.geometricResinVolumeMm3 - actual.volume()) > Math.max(1e-5, actual.volume() * 1e-9))
					throw "Fill geometric resin volume differs from actual foundation CAD volume";
				var region = CastingMoldCad.source(p);
				resources.push(region);
				var missing = actual.subtractAll([region]);
				resources.push(missing);
				var extra = region.subtractAll([actual]);
				resources.push(extra);
				difference = missing.volume() + extra.volume();
				if (!Math.isFinite(difference) || difference > Math.max(1e-5, actual.volume() * 1e-9))
					throw "Fill-region/actual foundation symmetric difference: " + difference;
			} catch (error:Dynamic) {
				for (part in resources)
					part.close();
				throw error;
			}
			for (part in resources)
				part.close();
			sys.io.File.saveContent(dir + "/fill-foundation-cad-check.json", ToolingCodec.canonical({
				sourceFingerprint: report.sourceFingerprint,
				reference: "foundation.body.geometry() transformed to mold coordinates",
				symmetricDifferenceMm3: difference
			}) + "\n");
			checks.push(FillAnalysis.check("process.fill.cad-volume-agreement", [p.document.input.id], ToolingCheckStatus.Pass,
				"Symmetric difference against actual foundation.body.geometry() passed; see fill-foundation-cad-check.json.",
				[report.sourceFingerprint, "fill-foundation-cad-check.json"]));
		}
		sys.io.File.saveContent(dir + "/casting-process-spec.json", materia.tooling.process.ProcessSpecCodec.encode(report.spec));
		sys.io.File.saveContent(dir + "/fill-analysis.json", ToolingCodec.canonical(report.fill) + "\n");
		writePour(report, dir);
		sys.io.File.saveContent(dir + "/cure-analysis.json", ToolingCodec.canonical(report.cure) + "\n");
		sys.io.File.saveContent(dir + "/castability-analysis.json", ToolingCodec.canonical(report.castability) + "\n");
		var text = [
			"# Casting design feedback",
			"",
			"Review suggestions only. Source geometry is unchanged; cure, CAD and physical release checks remain independent.",
			"",
			"All dimensions/points are in mold coordinates (mm).",
			""
		];
		var junction = report.sections.junction;
		text.push("Largest inscribed diameter: " + junction.lowerDiameterMm + ".." + junction.upperDiameterMm + " mm; numerical tolerance reached: "
			+ junction.converged + ". Witness: " + ToolingCodec.canonical(junction.witness) + "; nearby source faces: "
			+ junction.witnessSourceIds.join(", ") + ".");
		text.push("Declared slab geometry covers "
			+ report.thermalCoverage.coveredVolumeMm3
			+ " of "
			+ report.thermalCoverage.totalVolumeMm3
			+ " mm3; "
			+ report.thermalCoverage.uncoveredVolumeMm3
			+ " mm3 remains unbound. Junction heat flow and cure evidence are separate requirements.\n");
		for (v in report.castability.validation)
			text.push("- `" + v.rule + "` (" + v.status + ") [" + v.affectedIds.join(", ") + "]: " + v.message);
		text.push("\n## Suggested actions\n");
		for (proposal in report.castability.proposals) {
			text.push("- `" + proposal.id + "` — " + proposal.action + " (" + proposal.owner + "), sources: " + proposal.sourceIds.join(", ")
				+ "; blocking rule: `" + proposal.rule + "`. " + proposal.instruction);
			if (proposal.measuredMm != null || proposal.targetMm != null)
				text.push("  Measured: " + proposal.measuredMm + " mm; target: " + proposal.targetMm + " mm; axis: " + proposal.axis + ".");
			if (proposal.point != null)
				text.push("  Candidate point: " + ToolingCodec.canonical(proposal.point) + ".");
		}
		sys.io.File.saveContent(dir + "/castability-review.md", text.join("\n") + "\n");
		sys.io.File.saveContent(dir + "/junction-thickness.json", ToolingCodec.canonical(report.sections.junction) + "\n");
		sys.io.File.saveContent(dir + "/thermal-coverage.json", ToolingCodec.canonical(report.thermalCoverage) + "\n");
		sys.io.File.saveContent(dir + "/section-geometry.json", ToolingCodec.canonical(report.sections) + "\n");
		sys.io.File.saveContent(dir + "/process-diagnostics.json", ToolingCodec.canonical(ToolingDiagnostics.fromValidation(checks).items) + "\n");
		return checks;
	}

	static function writePour(report:materia.tooling.process.CastingProcessAnalysis.CastingProcessReport, dir:String):Void {
		var pour = report.pour;
		sys.io.File.saveContent(dir + "/pour-plan.json", ToolingCodec.canonical(pour) + "\n");
		var quantities:Array<Array<String>> = [["ingredient_id", "material_reference", "mass_kg"]];
		for (q in pour.quantities) quantities.push([q.ingredientId, q.materialReference, Std.string(q.massKg)]);
		sys.io.File.saveContent(dir + "/pour-quantities.csv", materia.tooling.export.ToolingFabrication.csv(quantities));
		var batches:Array<Array<String>> = [["batch", "mass_kg", "from_level_mm", "to_level_mm",
			"mixing_start_s", "pour_start_s", "pour_end_s"]];
		var batchQuantities:Array<Array<String>> = [["batch", "ingredient_id", "material_reference", "mass_kg"]];
		var text = ["# Pour plan", "", "Review calculation; process and physical release checks remain in process-diagnostics.json.",
			"", "Total wet mix: " + pour.totalMixMassKg + " kg.", ""];
		if (report.spec.mixComposition == null)
			text.push("Weighed material quantities are pending the declared mix fractions.");
		else text.push("Weighing quantities: pour-quantities.csv. The cure heat-denominator mass is " + pour.cureBasisMassKg + " kg.");
		var schedule = pour.schedule;
		if (schedule == null)
			text.push("Batch schedule is unresolved; see the validation records below.");
		else {
			for (batch in schedule.batches) {
				batches.push([Std.string(batch.index), Std.string(batch.massKg), Std.string(batch.lowerHeightMm), Std.string(batch.upperHeightMm),
					Std.string(batch.mixingStartSeconds), Std.string(batch.pourStartSeconds), Std.string(batch.pourEndSeconds)]);
				for (q in batch.quantities)
					batchQuantities.push([Std.string(batch.index), q.ingredientId, q.materialReference, Std.string(q.massKg)]);
			}
			text.push("Batch sequence: pour-batches.csv. " + schedule.batches.length + " batches; elapsed "
				+ schedule.totalElapsedSeconds + " s; minimum declared working-time margin " + schedule.minimumWorkingTimeMarginSeconds + " s.");
			text.push("Individual batch weighings: pour-batch-quantities.csv.");
		}
		text.push("");
		for (check in pour.validation) text.push("- `" + check.rule + "` (" + check.status + "): " + check.message);
		sys.io.File.saveContent(dir + "/pour-batches.csv", materia.tooling.export.ToolingFabrication.csv(batches));
		sys.io.File.saveContent(dir + "/pour-batch-quantities.csv", materia.tooling.export.ToolingFabrication.csv(batchQuantities));
		sys.io.File.saveContent(dir + "/pour-plan.md", text.join("\n") + "\n");
	}
}
