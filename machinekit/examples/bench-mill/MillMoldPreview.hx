import haxe.Json;
import materia.tooling.casting.CastingMoldPlanner;

class MillMoldPreview {
	/** Both source-project mold variants use the same setup and native planner as exports. */
	public static function scene(frame:BenchMillFrame, casting:MillMoldSetup.MillMoldCasting):haxe.io.Bytes {
		var document = MillMoldSetup.document(frame,CastingShopRecipe.BOARD_THICKNESS_MM,casting);
		var plan = machinekit.tooling.CastingMoldCad.plan(document,MillMoldSetup.options(frame,casting));
		return machinekit.tooling.CastingMoldExport.artifact(plan);
	}

	public static function main():Void {
		var frame = MillProject.smallFrame();
		var casting:MillMoldSetup.MillMoldCasting = Sys.args().indexOf("--column") >= 0 ? Column : Base;
		var input = MillMoldSetup.input(frame,casting), stem = casting == Column ? "column" : "base";
		var document = MillMoldSetup.document(frame,CastingShopRecipe.BOARD_THICKNESS_MM,casting);
		var options = MillMoldSetup.options(frame,casting);
		if (Sys.args().indexOf("--fabrication") >= 0) {
			MillFabricationPreview.write(document,options,casting == Column ? frame.column : frame.base,
				stem,"mill-"+stem+"-process-review");
			return;
		}
		var cwd = Sys.getCwd(),
			root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/bench-mill" : cwd;
		var dir = root + "/build/tooling/"+stem;
		sys.FileSystem.createDirectory(root + "/build/tooling");
		sys.FileSystem.createDirectory(dir);
		CastingMoldPlanner.setGeometry(new machinekit.tooling.NativeCastingMoldKernel());
		var sourceReview = CastingMoldPlanner.assess(document, options);
		if (sourceReview.plan == null) {
			var reportDir = root + "/build/tooling/"+stem+"-source-assessment/" + sourceReview.sourceFingerprint;
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
			var reportDir = root + "/build/tooling/"+stem+"-source-assessment/" + sourceReview.sourceFingerprint;
			machinekit.tooling.ToolingFiles.directory(reportDir);
			sys.io.File.saveContent(reportDir + "/review.json",
				Json.stringify({validation: plan.validation, diagnostics: assessment.diagnostics.items, fabrication: plan.fabrication}, null, "\t"));
			Sys.println("Unsupported tooling strategy: " + reportDir + "; no current mold geometry was generated.");
			return;
		}
		sys.io.File.saveContent(dir + "/source-input.json", materia.tooling.ToolingCodec.canonical(input) + "\n");
		sys.io.File.saveContent(dir + "/review.json", Json.stringify(review, null, "\t") + "\n");
		if (casting == Base && Sys.args().indexOf("--checks") >= 0)
			MillMoldChecks.run(frame, plan, review);
		machinekit.tooling.CastingMoldExport.write(plan, dir, review);
		Sys.println(stem+" mold plan: "
			+ plan.cores.length
			+ " cores, "
			+ plan.formers.length
			+ " formers, "
			+ plan.fixtures.length
			+ " datum fixtures");
	}
}
