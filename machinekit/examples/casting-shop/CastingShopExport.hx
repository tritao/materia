import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingLoads;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.CastingMoldExport;
import machinekit.tooling.ToolingFabricationExport;
import machinekit.tooling.ToolingPackageManifest;
import camkit.CamToolingPanels;
import cnckit.CncWriter;
import cnckit.CncController;

/** End-to-end phase 5 package; generation never grants fabrication approval. */
class CastingShopExport {
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

	static function processSpec(recipe:materia.tooling.ToolingRecipe,id:String):materia.tooling.process.CastingProcessSpec {
		var args = Sys.args(), index = args.indexOf("--process-spec");
		if (index < 0) {
			var result=CastingShopRecipe.process(recipe,id);result.id=id;
			return result;
		}
		if (index + 1 >= args.length || StringTools.startsWith(args[index + 1], "--") || args.lastIndexOf("--process-spec") != index)
			throw "Expected one --process-spec JSON path";
		var path = args[index + 1];
		var text=sys.io.File.getContent(path);
		if (haxe.io.Bytes.ofString(text).length>2097152) throw "Process shop record exceeds size limit";
		return materia.tooling.process.CastingProcessSpec.ProcessSpecs.fromRecord(haxe.Json.parse(text));
	}


	public static function write(document:materia.tooling.ToolingDocument,
			options:materia.tooling.casting.CastingMoldPlan.CastingMoldOptions,
			foundation:machinekit.structural.MineralFoundation,stem:String,processId:String,root:String,
			?extraChecks:materia.tooling.casting.CastingMoldPlan->machinekit.tooling.CastingMoldCad.CastingMoldReview->Void,
			?sourceReview:materia.tooling.casting.CastingMoldPlan->Array<ToolingValidation>):Void {
		var recordsOnly = Sys.args().indexOf("--records-only") >= 0;
		var name = stem+"-fabrication"+(recordsOnly ? "-records" : "");
		var destination = root + "/build/tooling/" + name,
			dir = destination + "-stage";
		removeGenerated(dir);
		machinekit.tooling.ToolingFiles.directory(dir);
		var p = machinekit.tooling.CastingMoldCad.plan(document,options);
		var process = processSpec(p.document.recipe,processId);
		var fabricationSpec = CastingShopRecipe.fabrication();
		var processReport = materia.tooling.process.CastingProcessAnalysis.analyze(p, process, fabricationSpec.gravityMps2, fabricationSpec,machinekit.tooling.CastingFillAnalysis.fromPlan,machinekit.tooling.CastingWallReview.fromPlan);
		var f = ToolingFabrication.plan(p, fabricationSpec, processReport);
		if (process.recipeFingerprint != ToolingCodec.recipeFingerprint(p.document.recipe))
			throw "Stale process recipe fingerprint";
		var checks:Array<ToolingValidation> = recordsOnly ? CastingMoldCad.corePanelFit(p) : [];
		var geometricReview:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview> = null;
		if (!recordsOnly) {
			var review = CastingMoldCad.review(p);
			geometricReview = review;
			checks = checks.concat(review.validation.filter(record ->
				StringTools.startsWith(record.rule, "tooling.cad.core-panel-fit.")));
			if (review.fingerprint != f.sourceFingerprint)
				throw "Stale geometric review";
			if (Sys.args().indexOf("--checks") >= 0 && extraChecks!=null)
				extraChecks(p,review);
			CastingMoldExport.write(p, dir, review);
			machinekit.tooling.MoldPlayback.write(p, review, dir);
			checks.push(ToolingLoads.check("fabrication.geometry-and-sequence", [p.document.input.id], ToolingCheckStatus.Pass,
				"Exact source reconstruction, boundary coverage, assembly/demolding sweeps and portable assembly restoration passed.", [review.fingerprint]));
		} else {
			checks.push(ToolingLoads.check("fabrication.geometry-and-sequence", [p.document.input.id], ToolingCheckStatus.Pending,
				"Records-only export did not run CAD/sequence validation.", []));
		}
		var panels = ToolingFabricationExport.panels(p, f),
			settings = CastingShopRecipe.cam(panels);
		var formerShop=machinekit.tooling.FormerShopExport.build(p);
		for (former in formerShop.formers) for (board in former.boards) {
			var machining=board.machining;
			if (machining==null) continue;
			try {
				for (setup in camkit.CamToolingFormers.build(machining,settings)) {
					var file=setup.id+".ngc";
					var plane=settings.spoilboardPlane;
					if (plane==null) throw "Former CAM has no declared spoilboard plane";
					var code=CncWriter.write(setup.program,new CncController(cnckit.CncDialect.LinuxCnc,setup.program.tools),
						settings.travelSpace==MachineCoordinates ? settings.travel : null);
					sys.io.File.saveContent(dir+"/"+file,"(REVIEW ONLY - former stock, machine, workholding and sealing qualification pending)\n"
						+"(G54 uses the declared fixed work datum; spoilboard Z "+(plane*1000)+" mm; stock top Z "
						+((plane+board.stock.measuredThickness*0.001)*1000)+" mm)\n"+code);
					board.programs.push(file);
				}
			} catch (error:Dynamic) {
				var message="Former "+board.id+" CAM rejected: "+Std.string(error);
				board.reasons.push(message);
				var validation=formerShop.validation.filter(row -> row.rule=="fabrication.former-machining."+former.id)[0];
				validation.message=validation.status==Fail ? validation.message+"; "+message : message;
				validation.status=Fail;validation.severity=materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
				if (validation.affectedIds.indexOf(board.id)<0) validation.affectedIds.push(board.id);
			}
		}
		for (former in formerShop.formers) {
			var validation=formerShop.validation.filter(row -> row.rule=="fabrication.former-machining."+former.id)[0];
			if (validation.status==Pending && former.boards.length>0
				&& former.boards.filter(board -> board.programs.length==0).length==0) {
				validation.status=Pass;
				validation.severity=materia.tooling.ToolingDiagnostic.ToolingSeverity.Info;
				validation.message="Exact board construction and controller CAM generated; cutter fit and access checked. Machine, workholding, board/grip and sealing qualification remain Pending in fabrication process checks.";
			}
		}
		checks = checks.concat(ToolingFabricationExport.write(p, f, dir,formerShop));
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
				if (Sys.args().indexOf("--checks") >= 0) {
					var actual = [for (setup in setups) for (id in setup.operationIds) id];
					var expected = [for (operation in panel.operations) operation.id];
					var ordered = actual.copy();
					actual.sort(Reflect.compare);
					expected.sort(Reflect.compare);
					if (ToolingCodec.canonical(actual) != ToolingCodec.canonical(expected))
						throw "Panel CAM must preserve every source operation exactly once";
					var outside = panel.operations.filter(operation -> operation.kind == OutsideProfile);
					if (outside.length != 1 || ordered[ordered.length - 1] != outside[0].id)
						throw "Panel outside profile must be the final operation";
				}
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
			"# Panel machining review\n\nPrograms are LinuxCNC review outputs in mm. Settings originate in metres and m/s. Do not substitute relief cutters without regenerating the plan.\n\nEach program uses G54 with Z0 at the face being machined. XY starts at the raw blank lower left; the finished bounding profile is offset by the declared blank margin. Negative-Z setups reflect X about the finished profile midline: flip about local Y and re-establish the origin. Operation order clears named access pockets first and releases the outside profile last, after other faces. CAM checks supplied stock, fixtures and travel; an empty fixture list does not qualify workholding.\n\nRetain the part for the final contour with a qualified fixture/spoilboard or tabs specified by a revised operation plan. Through cuts include the declared breakthrough allowance into the declared spoilboard. Execute split single-tool programs in the cam-programs.json order, preserving access pockets and outside-last. Final outside cuts use the declared onion skin followed by its separate removal pass, or an evidenced vacuum area/holding-force assessment. G49 single-tool programs require touching off Z with that tool; no length offset applies. Fit-coupon programs and locator-coupon programs use the same settings. Resolve every blocking fabrication-validation.json record before fabrication release.\n"
			+"\n## Release before assembly\n\nUse release-system.md and release-operations.csv to seal exposed board faces, cure, apply the declared wax/PVA sequence and dry all tooling before fitting source steel. Keep insert/anchor bonding faces uncoated and the listed adjacent contacts masked. Physical chemical coupon evidence remains required.\n\n## Former boards\n\nUse former-board-stock.csv for each single-board or hollow-box part, former-pocket-screws.csv for dry-side jig drilling and closure, and former-manufacturing.json for sealing faces and each board's program order. The per-former assembly files show the removable board construction. Neck and flange sheets register against the wall and use its external closure; hollow-body joints use internal pocket screws.\n"
			+"\nFormer programs retain the declared fixed G54 work datum and spoilboard plane; each header states its actual stock-top Z. Thicker blanks therefore use a different top Z, not a different assumed datum. Machine and seal all listed contact surfaces; pocket-screw heads and access stay inside the dry cavity. Board/grip, jig, workholding and machine qualification remain pending.\n");
		checks = checks.concat(machinekit.tooling.ToolingProcessExport.write(p,process,f.spec.gravityMps2,dir,
			geometricReview,f.spec,foundation,processReport,p.document.input.interfaceContext));
		if(sourceReview!=null)checks=checks.concat(sourceReview(p));
		ToolingPackageManifest.write(dir, f, checks, p, process);
		ToolingPackageManifest.verify(dir);
		Sys.println("Fabrication package integrity check passed");
		removeGenerated(destination);
		sys.FileSystem.rename(dir,destination);
		var formerProgramCount=0;
		for (former in formerShop.formers) for (board in former.boards) formerProgramCount+=board.programs.length;
		Sys.println("Fabrication review package: "
			+ destination
			+ "; "
			+ (programs.length+formerProgramCount)
			+ " review programs ("+programs.length+" panel, "+formerProgramCount+" former); readiness remains separately reported in manifest.json");
	}
}
