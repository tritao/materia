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
			review:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview>):Array<ToolingValidation> {
		var report = CastingProcessAnalysis.analyze(p, spec, gravityMps2);
		var checks = report.fill.validation.concat(report.cure.validation);
		if (review == null)
			checks.push(FillAnalysis.check("process.fill.cad-volume-agreement", [p.document.input.id], ToolingCheckStatus.Pending,
				"Records-only fill geometry has not been cross-checked against reconstructed/source CAD.", []));
		else {
			if (review.fingerprint != report.sourceFingerprint)
				throw "Stale process CAD review";
			var cadSourceVolumeMm3 = review.sourceVolumeMm3;
			if (!Math.isFinite(cadSourceVolumeMm3) || cadSourceVolumeMm3 <= 0)
				throw "Invalid process CAD volume";
			var difference = Math.abs(cadSourceVolumeMm3 - report.fill.totalMixVolumeMm3);
			if (difference > Math.max(1e-5, cadSourceVolumeMm3 * 1e-9))
				throw "Fill-region/CAD volume differs: " + difference;
			checks.push(FillAnalysis.check("process.fill.cad-volume-agreement", [p.document.input.id], ToolingCheckStatus.Pass,
				"Exact orthogonal resin-region volume agrees with the source/reconstruction CAD review.", [report.sourceFingerprint]));
		}
		sys.io.File.saveContent(dir + "/casting-process-spec.json", materia.tooling.process.ProcessSpecCodec.encode(report.spec));
		sys.io.File.saveContent(dir + "/fill-analysis.json", ToolingCodec.canonical(report.fill) + "\n");
		sys.io.File.saveContent(dir + "/cure-analysis.json", ToolingCodec.canonical(report.cure) + "\n");
		sys.io.File.saveContent(dir + "/process-diagnostics.json", ToolingCodec.canonical(ToolingDiagnostics.fromValidation(checks).items) + "\n");
		return checks;
	}
}
