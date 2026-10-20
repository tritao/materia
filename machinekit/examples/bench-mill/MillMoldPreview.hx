import haxe.Json;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;

class MillMoldPreview {
	public static function main():Void {
		var frame = MillMoldSetup.frame(), input = MillMoldSetup.input(frame);
		var cwd = Sys.getCwd(),
			root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/bench-mill" : cwd;
		var dir = root + "/build/tooling/base";
		sys.FileSystem.createDirectory(root + "/build/tooling");
		sys.FileSystem.createDirectory(dir);
		var document = MillMoldSetup.document(frame);
		var sourceReview = CastingMoldPlanner.assess(document, MillMoldSetup.options(frame));
		if (sourceReview.plan == null) {
			var reportDir = root + "/build/tooling/base-source-assessment/" + sourceReview.sourceFingerprint;
			machinekit.tooling.ToolingFiles.directory(reportDir);
			sys.io.File.saveContent(reportDir + "/source-document.json", materia.tooling.ToolingCodec.encode(document));
			sys.io.File.saveContent(reportDir + "/source-assessment.json", Json.stringify(sourceReview, null, "\t") + "\n");
			Sys.println("Unsupported source geometry: " + reportDir + "; no current mold geometry was generated.");
			return;
		}
		var plan:materia.tooling.casting.CastingMoldPlan = cast sourceReview.plan;
		var assessment = machinekit.tooling.CastingMoldCad.assess(plan),
			review = assessment.geometry;
		if (review == null) {
			var reportDir = root + "/build/tooling/base-source-assessment/" + sourceReview.sourceFingerprint;
			machinekit.tooling.ToolingFiles.directory(reportDir);
			sys.io.File.saveContent(reportDir + "/mold-plan.json", CastingMoldCodec.encode(plan));
			sys.io.File.saveContent(reportDir + "/review.json",
				Json.stringify({validation: plan.validation, diagnostics: assessment.diagnostics.items, fabrication: plan.fabrication}, null, "\t"));
			Sys.println("Unsupported tooling strategy: " + reportDir + "; no current mold geometry was generated.");
			return;
		}
		sys.io.File.saveContent(dir + "/source-input.json", Json.stringify(input, null, "\t") + "\n");
		sys.io.File.saveContent(dir + "/review.json", Json.stringify(review, null, "\t") + "\n");
		if (Sys.args().indexOf("--checks") >= 0)
			MillMoldChecks.run(frame, plan, review);
		machinekit.tooling.CastingMoldExport.write(plan, dir, review);
		var encoded = CastingMoldCodec.encode(plan);
		CastingMoldCodec.decode(encoded);
		sys.io.File.saveContent(dir + "/mold-plan.json", encoded);
		Sys.println("Base mold plan: "
			+ plan.cores.length
			+ " cores, "
			+ plan.formers.length
			+ " formers, "
			+ plan.fixtures.length
			+ " datum fixtures");
	}
}
