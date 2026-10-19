import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingLoads;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.CastingMoldExport;
import machinekit.tooling.ToolingFabricationExport;
import machinekit.tooling.ToolingPackageManifest;
import camkit.CamToolingPanels;
import cnckit.CncWriter;
import cnckit.CncController;

/** End-to-end phase 5 package; generation never grants fabrication approval. */
class MillFabricationPreview {
	static function removeGenerated(path:String):Void {
		if (path.indexOf("/build/tooling/") < 0)
			throw "Generated package directory must be below build/tooling";
		if (!sys.FileSystem.exists(path))
			return;
		for (name in sys.FileSystem.readDirectory(path)) {
			var file = path + "/" + name;
			if (sys.FileSystem.isDirectory(file))
				removeGenerated(file);
			else
				sys.FileSystem.deleteFile(file);
		}
		sys.FileSystem.deleteDirectory(path);
	}

	static function data(dir:String, name:String, value:Dynamic):Void
		sys.io.File.saveContent(dir + "/" + name, ToolingCodec.canonical(value) + "\n");

	static function processSpec(recipe:materia.tooling.ToolingRecipe):materia.tooling.process.CastingProcessSpec {
		var args = Sys.args(), index = args.indexOf("--process-spec");
		if (index < 0)
			return MillFabricationSetup.process(recipe);
		if (index + 1 >= args.length || StringTools.startsWith(args[index + 1], "--") || args.lastIndexOf("--process-spec") != index)
			throw "Expected one --process-spec JSON path";
		var path = args[index + 1];
		return materia.tooling.process.ProcessSpecCodec.decode(sys.io.File.getContent(path));
	}

	public static function main():Void {
		var cwd = Sys.getCwd(),
			root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/bench-mill" : cwd;
		var recordsOnly = Sys.args().indexOf("--records-only") >= 0;
		var name = recordsOnly ? "base-fabrication-records" : "base-fabrication";
		var destination = root + "/build/tooling/" + name,
			dir = destination + "-stage";
		removeGenerated(dir);
		machinekit.tooling.ToolingFiles.directory(dir);
		var frame = MillMoldSetup.frame(),
			p = CastingMoldPlanner.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
		var f = ToolingFabrication.plan(p, MillFabricationSetup.spec());
		var process = processSpec(p.document.recipe);
		if (process.recipeFingerprint != ToolingCodec.recipeFingerprint(p.document.recipe))
			throw "Stale process recipe fingerprint";
		materia.tooling.process.CastingProcessAnalysis.validateSectionSources(p.document, process);
		var checks:Array<ToolingValidation> = [];
		var geometricReview:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview> = null;
		if (!recordsOnly) {
			var review = CastingMoldCad.review(p);
			geometricReview = review;
			if (review.fingerprint != f.sourceFingerprint)
				throw "Stale geometric review";
			if (Sys.args().indexOf("--checks") >= 0)
				MillMoldChecks.run(frame, p, review);
			CastingMoldExport.write(p, dir, review);
			machinekit.tooling.MoldPlayback.write(p, review, dir);
			checks.push(ToolingLoads.check("fabrication.geometry-and-sequence", [p.document.input.id], ToolingCheckStatus.Pass,
				"Exact source reconstruction, boundary coverage, assembly/demolding sweeps and portable assembly restoration passed.", [review.fingerprint]));
		} else {
			sys.io.File.saveContent(dir + "/mold-plan.json", CastingMoldCodec.encode(p));
			checks.push(ToolingLoads.check("fabrication.geometry-and-sequence", [p.document.input.id], ToolingCheckStatus.Pending,
				"Records-only export did not run CAD/sequence validation.", []));
		}
		ToolingFabricationExport.write(p, f, dir);
		var panels = ToolingFabricationExport.panels(p, f),
			settings = MillFabricationSetup.cam(panels);
		if (Math.abs(settings.blankMargin * 1000 - f.spec.blankMarginMm) > 1e-7)
			throw "CAM margin differs from blank requirements";
		data(dir, "cam-settings.json", {
			lengthUnit: "m",
			speedUnit: "m/s",
			studyOnly: true,
			chordTolerance: settings.chordTolerance,
			safeZ: settings.safeZ,
			spindleRpm: settings.spindleRpm,
			feed: settings.feed,
			plungeFeed: settings.plungeFeed,
			stepDown: settings.stepDown,
			stepOverFraction: settings.stepOverFraction,
			blankMargin: settings.blankMargin,
			tools: [
				for (t in settings.tools)
					{
						number: t.number,
						diameter: t.diameter,
						length: t.length
					}
			],
			fixtures: settings.fixtures,
			travel: {lower: settings.travel.lower, upper: settings.travel.upper},
			workOrigin: settings.workOrigin,
			workholdingReference: settings.workholdingReference
		});
		var programs:Array<Dynamic> = [];
		for (panel in panels) {
			try {
				var setups = CamToolingPanels.build(panel, settings);
				for (setup in setups) {
					var name = setup.id + ".ngc",
						code = CncWriter.write(setup.program, new CncController(), settings.travel);
					sys.io.File.saveContent(dir + "/" + name, "(REVIEW ONLY - unqualified study cutting data and workholding)\n" + code);
					programs.push({
						panelId: panel.id,
						file: name,
						operationIds: setup.operationIds,
						side: setup.side,
						sourceSetupId: setup.sourceSetupId,
						reflectedX: setup.reflectedX,
						sourceMinX: setup.sourceMinX,
						sourceMinY: setup.sourceMinY,
						widthMm: setup.widthMm,
						heightMm: setup.heightMm,
						stock: setup.setup.stock,
						workOrigin: setup.setup.workOrigin,
						tools: [
							for (t in settings.tools)
								{
									number: t.number,
									diameter: t.diameter,
									length: t.length
								}
						]
					});
				}
				checks.push(ToolingLoads.check("fabrication.cam." + panel.id, [panel.id], ToolingCheckStatus.Pass,
					"Panel operations mapped one-to-one and checked against declared stock, fixtures and travel before LinuxCNC export.",
					[f.sourceFingerprint]));
			} catch (e:Dynamic) {
				checks.push(ToolingLoads.check("fabrication.cam." + panel.id, [panel.id], ToolingCheckStatus.Fail, "CAM export rejected: " + Std.string(e),
					[]));
			}
		}
		checks.push(ToolingLoads.check("fabrication.cam.shop-qualification", [p.document.recipe.id], ToolingCheckStatus.Pending,
			"Study tools/feeds/travel and the workholding/part-release sequence require an actual machine, tooling and shop trial.",
			settings.workholdingReference == null ? [] : [settings.workholdingReference]));
		data(dir, "cam-programs.json", programs);
		sys.io.File.saveContent(dir + "/machining-instructions.md",
			"# Panel machining review\n\nPrograms are LinuxCNC review outputs in mm. Settings originate in metres and m/s. Do not substitute relief cutters without regenerating the plan.\n\nEach program uses G54 with Z0 at the face being machined. XY starts at the raw blank lower left; the finished bounding profile is offset by the declared blank margin. Negative-Z setups reflect X about the finished profile midline: flip about local Y and re-establish the origin. Operation order clears named access pockets first and releases the outside profile last, after other faces. CAM checks supplied stock, fixtures and travel; an empty fixture list does not qualify workholding.\n\nRetain the part for the final contour with a qualified fixture/spoilboard or tabs specified by a revised operation plan. Through cuts stop at the declared stock bottom; no undeclared breakthrough allowance is applied. Fit-coupon programs and locator-coupon programs use the same settings. Resolve every blocking fabrication-validation.json record before fabrication release.\n");
		checks = checks.concat(machinekit.tooling.ToolingProcessExport.write(p, process, f.spec.gravityMps2, dir, geometricReview));
		MillMoldStudy.write(MillMoldStudy.study(frame, geometricReview, !recordsOnly, process), dir);
		ToolingPackageManifest.write(dir, f, checks);
		ToolingPackageManifest.verify(dir);
		if (Sys.args().indexOf("--checks") >= 0) {
			var path = dir + "/coupon-trial.csv",
				saved = sys.io.File.getContent(path),
				rejected = false;
			sys.io.File.saveContent(path, saved + "tampered");
			try
				ToolingPackageManifest.verify(dir)
			catch (e:Dynamic)
				rejected = Std.string(e).indexOf("Tooling artifact hash differs") >= 0;
			sys.io.File.saveContent(path, saved);
			if (!rejected)
				throw "Manifest failed to reject a modified coupon record";
			ToolingPackageManifest.verify(dir);
			Sys.println("Fabrication manifest integrity/tamper checks passed");
		}

		removeGenerated(destination);
		sys.FileSystem.rename(dir, destination);
		Sys.println("Fabrication review package: "
			+ destination
			+ "; "
			+ programs.length
			+ " checked programs; readiness remains separately reported in manifest.json");
	}
}
