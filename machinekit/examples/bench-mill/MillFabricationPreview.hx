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
		var process = processSpec(p.document.recipe);
		var fabricationSpec = MillFabricationSetup.spec();
		var processReport = materia.tooling.process.CastingProcessAnalysis.analyze(p, process, fabricationSpec.gravityMps2, fabricationSpec);
		var f = ToolingFabrication.plan(p, fabricationSpec, processReport.cure);
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
		checks = checks.concat(ToolingFabricationExport.write(p, f, dir));
		var panels = ToolingFabricationExport.panels(p, f),
			settings = MillFabricationSetup.cam(panels);
		if (Math.abs(settings.blankMargin * 1000 - f.spec.blankMarginMm) > 1e-7)
			throw "CAM margin differs from blank requirements";
		data(dir, "cam-settings.json", {
			lengthUnit: "m",
			speedUnit: "m/s",
			studyOnly: true,
			chordTolerance: settings.chordTolerance,
			arcTolerance: settings.arcTolerance,
			blendTolerance: settings.blendTolerance,
			cutDirection: settings.cutDirection,
			spindleSpinUpSeconds: settings.spindleSpinUpSeconds,
			nominalDiameters: settings.nominalDiameters,
			diameterMatchTolerance: settings.diameterMatchTolerance,
			toolMeasurementReference: settings.toolMeasurementReference,
			singleToolPrograms: settings.singleToolPrograms,
			spoilboardPlane: settings.spoilboardPlane,
			breakthroughAllowance: settings.breakthroughAllowance,
			vacuumHold: settings.vacuumHold,
			onionSkinDepth: settings.onionSkinDepth,
			approachClearance: settings.approachClearance,
			drillPeckRatio: settings.drillPeckRatio,
			drillPeckDepth: settings.drillPeckDepth,
			travelSpace: settings.travelSpace,
			workOriginReference: settings.workOriginReference,
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
						nominalDiameter: t.nominalDiameter,
						fluteLength: t.fluteLength,
						stickOut: t.stickOut,
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
				if (panel.id == "bed" && Sys.args().indexOf("--checks") >= 0 && setups.length != 4)
					throw "Bed requires exactly four precedence-constrained tool loads";
				var pending = [for (setup in setups) for (reason in setup.readinessPending) reason];
				for (setup in setups) {
					var name = setup.id + ".ngc",
						code = CncWriter.write(setup.program, new CncController(cnckit.CncDialect.LinuxCnc, setup.program.tools),
							settings.travelSpace == MachineCoordinates ? settings.travel : null);
					if (Sys.args().indexOf("--checks") >= 0 && settings.singleToolPrograms) {
						if (code.indexOf("touch off Z with this tool; no length offset") < 0 || code.indexOf("G43") >= 0)
							throw "Shipped G49 program must declare per-tool touch-off without length offsets";
					}
					sys.io.File.saveContent(dir + "/" + name, "(REVIEW ONLY - unqualified study cutting data and workholding)\n" + code);
					programs.push({
						panelId: panel.id,
						file: name,
						operationIds: setup.operationIds,
						side: setup.side,
						sourceSetupId: setup.sourceSetupId,
						readinessPending: setup.readinessPending,
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
									nominalDiameter: t.nominalDiameter,
									fluteLength: t.fluteLength,
									stickOut: t.stickOut,
									length: t.length
								}
						]
					});
				}
				checks.push(ToolingLoads.check("fabrication.cam." + panel.id, [panel.id],
					pending.length == 0 ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
					(pending.length == 0 ? "Panel operations checked in machine coordinates with measured tools, fixtures and retention." : pending.join("; ")),
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
			"# Panel machining review\n\nPrograms are LinuxCNC review outputs in mm. Settings originate in metres and m/s. Do not substitute relief cutters without regenerating the plan.\n\nEach program uses G54 with Z0 at the face being machined. XY starts at the raw blank lower left; the finished bounding profile is offset by the declared blank margin. Negative-Z setups reflect X about the finished profile midline: flip about local Y and re-establish the origin. Operation order clears named access pockets first and releases the outside profile last, after other faces. CAM checks supplied stock, fixtures and travel; an empty fixture list does not qualify workholding.\n\nRetain the part for the final contour with a qualified fixture/spoilboard or tabs specified by a revised operation plan. Through cuts include the declared breakthrough allowance into the declared spoilboard. Execute split single-tool programs in the cam-programs.json order, preserving access pockets and outside-last. Final outside cuts use the declared onion skin followed by its separate removal pass, or an evidenced vacuum area/holding-force assessment. G49 single-tool programs require touching off Z with that tool; no length offset applies. Fit-coupon programs and locator-coupon programs use the same settings. Resolve every blocking fabrication-validation.json record before fabrication release.\n");
		checks = checks.concat(machinekit.tooling.ToolingProcessExport.write(p, process, f.spec.gravityMps2, dir, geometricReview, f.spec, frame.base));
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
			var manifestPath = dir + "/manifest.json",
				original = sys.io.File.getContent(manifestPath);
			function rejectsManifest(change:Dynamic->Void, expected:String):Void {
				var altered:Dynamic = haxe.Json.parse(original);
				change(altered);
				sys.io.File.saveContent(manifestPath, ToolingCodec.canonical(altered));
				var error = "";
				try
					ToolingPackageManifest.verify(dir)
				catch (e:Dynamic)
					error = Std.string(e);
				sys.io.File.saveContent(manifestPath, original);
				if (error.indexOf(expected) < 0)
					throw "Manifest tamper regression: " + error;
			}
			rejectsManifest(m -> m.fabrication.ready = true, "readiness differs");
			rejectsManifest(m -> m.replacementGroups = new Array<Dynamic>(), "package fingerprint differs");
			rejectsManifest(m -> {
				var records:Array<Dynamic> = cast m.validation;
				records[0].message = "altered validation";
			}, "validation differs");
			rejectsManifest(m -> {
				var records:Array<Dynamic> = cast m.validation;
				records[0].severity = "unknown";
			}, "severity/status consistency");
			rejectsManifest(m -> {
				var records:Array<Dynamic> = cast m.validation;
				records[0].status = "unknown";
			}, "validation status");
			rejectsManifest(m -> {
				var records:Array<Dynamic> = cast m.validation;
				records[0].status = "pass";
				records[0].severity = "error";
			}, "severity/status consistency");
			rejectsManifest(m -> m.version = 1, "Unsupported tooling manifest");
			ToolingPackageManifest.verify(dir);
			var unknown = new machinekit.tooling.ToolingBlock();
			if (ToolingFabricationExport.classify(unknown).classification != UnsupportedHardware)
				throw "Unknown hardware was silently ignored";
			var hardwarePath = dir + "/hardware-validation.json",
				originalHardware = sys.io.File.getContent(hardwarePath);
			data(dir, "hardware-validation.json", [
				ToolingLoads.check("fabrication.hardware.test-probe", ["test-probe"], ToolingCheckStatus.Unsupported, "Test unknown hardware", [])
			]);
			ToolingPackageManifest.write(dir, f, checks);
			var probe:Dynamic = haxe.Json.parse(sys.io.File.getContent(manifestPath));
			var probeValidation:Array<Dynamic> = cast probe.validation;
			if (probeValidation.filter(v -> v.rule == "fabrication.hardware.test-probe").length != 1 || probe.fabrication.ready == true)
				throw "Writer dropped unknown hardware because caller omitted returned checks";
			ToolingPackageManifest.verify(dir);
			sys.io.File.saveContent(hardwarePath, originalHardware);
			ToolingPackageManifest.write(dir, f, checks);
			ToolingPackageManifest.verify(dir);
			Sys.println("Fabrication manifest integrity/tamper checks passed");
		}

		removeGenerated(destination);
		sys.FileSystem.rename(dir, destination);
		Sys.println("Fabrication review package: "
			+ destination
			+ "; "
			+ programs.length
			+ " review programs; readiness remains separately reported in manifest.json");
	}
}
