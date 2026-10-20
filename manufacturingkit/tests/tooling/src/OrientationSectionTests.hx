import materia.casting.CastingFeature.CastingDimension;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.process.MoldOrientationStudy;
import materia.tooling.process.ProcessSpecCodec;

class OrientationSectionTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, expected:String):Void {
		var message = "";
		try
			fn()
		catch (error:Dynamic)
			message = Std.string(error);
		check(message.indexOf(expected) >= 0, "Expected " + expected + ", got " + message);
	}

	static function close(a:Float, b:Float, label:String):Void
		check(Math.abs(a - b) < 1e-6, label + ": " + a + " vs " + b);

	public static function run(p:CastingMoldPlan, s:materia.tooling.process.CastingProcessSpec):Void {
		var box = {
			x: 0.0,
			y: 0.0,
			z: 0.0,
			width: 100.0,
			depth: 60.0,
			height: 20.0
		};
		var split = [
			{
				x: 0.0,
				y: 0.0,
				z: 0.0,
				width: 30.0,
				depth: 60.0,
				height: 20.0
			},
			{
				x: 30.0,
				y: 0.0,
				z: 0.0,
				width: 70.0,
				depth: 60.0,
				height: 20.0
			}
		];
		var report = OrthogonalSections.analyze("body", "hash", split, []);
		check(report.spans.length == 3, "coplanar strips aggregate by physical endpoints");
		for (span in report.spans) {
			close(span.thicknessMm, span.axis == X ? 100 : span.axis == Y ? 60 : 20, "ray spans physical material, not subtraction fragments");
			close(span.volumeMm3, 120000, "each axis covers the whole block");
		}
		check(report.maximumInscribedThicknessMm == 20, "rectangular inscribed thickness exact");
		var shape = split.concat([
			{
				x: 100.0,
				y: 0.0,
				z: 0.0,
				width: 20.0,
				depth: 20.0,
				height: 20.0
			}
		]);
		var junction = OrthogonalSections.analyze("body", "hash", shape, []);
		check(junction.maximumInscribedThicknessMm == null, "nonrectangular junction never uses a chord as an inscribed diameter");
		check(junction.validation.filter(v -> v.rule == "process.sections.junction-thickness" && v.status == "pass").length == 1,
			"junction diameter has converged bounds without claiming thermal coverage");
		for (axis in junction.coveredVolumeByAxisMm3)
			close(axis.volumeMm3, 128000, "junction coverage exact");
		shape.reverse();
		check(ToolingCodec.canonical(OrthogonalSections.analyze("body", "hash", shape, [])) == ToolingCodec.canonical(junction),
			"section ordering independent of box order");
		var owners = OrthogonalSections.analyze("body", "hash", [box], [
			{
				id: "core",
				box: {
					x: 0,
					y: 0,
					z: 20,
					width: 100,
					depth: 60,
					height: 20
				}
			}
		]);
		check(owners.spans.filter(x -> x.axis == Z)[0].upperSourceIds[0] == "core", "opposing face preserves feature ID");
		var boundary:materia.tooling.process.CastingProcessSpec.ThermalBoundary = {
			kind: Insulated,
			temperatureC: null,
			heatTransferWPerM2K: null,
			evidence: []
		};
		var derived = OrthogonalSections.cureSection(owners.spans.filter(x -> x.axis == Z)[0], boundary, boundary, [], owners.geometryFingerprint);
		check(derived.thicknessMm == 20 && derived.sourceIds.indexOf("core") >= 0,
			"extracted thickness feeds the cure-section contract with explicit thermal boundaries");
		s = ProcessSpecCodec.snapshot(s);
		s.recipeFingerprint = ToolingCodec.recipeFingerprint(p.document.recipe);
		s.sections = [];
		var a:MoldOrientationCandidate = {
			id: "a",
			document: p.document,
			options: p.options,
			process: s
		};
		var b:MoldOrientationCandidate = {
			id: "b",
			document: p.document,
			options: p.options,
			process: s
		};
		var comparison = MoldOrientationStudy.run([b, a], FabricationTests.spec());
		check(comparison.rankedCandidates[0].id == "a", "orientation ties resolve by candidate ID");
		check(comparison.preferredReviewCandidateId == null, "pure planner cannot claim a CAD-validated preference");
		check(ToolingCodec.canonical(comparison) == ToolingCodec.canonical(MoldOrientationStudy.run([a, b], FabricationTests.spec())),
			"candidate-order permutation deterministic");
		var shared = materia.tooling.process.CastingProcessAnalysis.analyze(p, s, FabricationTests.spec().gravityMps2, FabricationTests.spec());
		check(ToolingCodec.canonical(comparison.rankedCandidates[0].fill) == ToolingCodec.canonical(shared.fill),
			"planned study uses exactly the shared process fill records");
		check(ToolingCodec.canonical(comparison.rankedCandidates[0].cure) == ToolingCodec.canonical(shared.cure),
			"planned study uses exactly the shared cure records");
		check(comparison.rankingRule.indexOf("trapped volume") >= 0 && comparison.rankedCandidates[0].decidingReason.length > 0,
			"ranking declares its tuple and deciding reason");
		var first = comparison.rankedCandidates[0],
			second = comparison.rankedCandidates[1];
		first.rankMetrics.failCount++;
		check(MoldOrientationStudy.compare(first, second) > 0, "failure count precedes process metrics");
		first.rankMetrics.failCount--;
		first.rankMetrics.openChangeRequests++;
		check(MoldOrientationStudy.compare(first, second) > 0, "open requests precede process metrics");
		first.rankMetrics.openChangeRequests--;
		first.rankMetrics.trappedVolumeMm3 = 1;
		second.rankMetrics.trappedVolumeMm3 = 2;
		check(MoldOrientationStudy.compare(first, second) < 0, "ranking minimises trapped volume first");
		second.rankMetrics.trappedVolumeMm3 = 1;
		first.rankMetrics.worstCureMarginC = 1;
		second.rankMetrics.worstCureMarginC = 2;
		check(MoldOrientationStudy.compare(first, second) > 0, "ranking maximises worst cure margin second");
		first.rankMetrics.worstCureMarginC = 2;
		first.rankMetrics.requiresFlip = true;
		second.rankMetrics.requiresFlip = false;
		check(MoldOrientationStudy.compare(first, second) > 0, "ranking then avoids a flip");
		first.rankMetrics.requiresFlip = false;
		first.rankMetrics.panels = 10;
		second.rankMetrics.panels = 9;
		check(MoldOrientationStudy.compare(first, second) > 0, "ranking then minimises panels");
		first.rankMetrics.panels = 9;
		first.rankMetrics.sheets = 2;
		second.rankMetrics.sheets = 1;
		check(MoldOrientationStudy.compare(first, second) > 0, "ranking then minimises sheets");
		var effort = comparison.rankedCandidates[0].effort;
		if (effort == null)
			throw "Missing planned effort";
		check(effort.panels == p.panels.members.length + p.supportPanels.length, "production panel effort excludes coupons");
		check(effort.castingMassKg == p.handling.castingMassKg, "orientation mass uses real handling plan");
		var invalid = ToolingCodec.snapshot(p.document);
		invalid.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported;
		var failed:MoldOrientationCandidate = {
			id: "failed",
			document: invalid,
			options: p.options,
			process: s
		};
		var rejected = MoldOrientationStudy.run([failed, a], FabricationTests.spec());
		check(rejected.rankedCandidates[0].id == "a", "complete plan ranks ahead of failed/unsupported candidate");
		var brokenOptions = materia.tooling.casting.CastingMoldOptionsCodec.snapshot(p.options);
		brokenOptions.anchorage = [];
		var broken:MoldOrientationCandidate = {
			id: "planner-fails",
			document: p.document,
			options: brokenOptions,
			process: s
		};
		var failure = MoldOrientationStudy.run([broken, a], FabricationTests.spec()).rankedCandidates[1];
		check(!failure.planningSucceeded && failure.planningFailure != null && failure.effort == null,
			"planning failure preserves source fill report without fabricated effort");
		check(failure.fill == null
			&& failure.validation.filter(v -> v.rule == "orientation.planner" && v.status == "unsupported").length == 1,
			"unplanned candidate has no second process pipeline");
		var changed = MoldOrientationStudy.reorient(p.document, "turned", materia.assembly.AssemblyFrames.identity());
		check(ToolingCodec.inputFingerprint(changed.input) == ToolingCodec.inputFingerprint(p.document.input),
			"reorientation cannot change the source revision");
		var metrics = MoldOrientationStudy.metrics(p, FabricationTests.spec());
		check(metrics.blankSheetUtilization > 0 && metrics.blankSheetUtilization <= 1, "bounding blank yield bounded");
		var rigid = ToolingCodec.snapshot(p.document);
		rigid.setup.sourceToMold = materia.assembly.AssemblyFrames.identity();
		rigid.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer;
		rigid.setup.features[0].withdrawal = {x: 1, y: 0, z: 0};
		var turn = MoldOrientationStudy.reorient(rigid, "turned-rigid",
			materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 0, [0, -1, 0, 1, 0, 0, 0, 0, 1]));
		var withdrawal = turn.setup.features[0].withdrawal;
		check(withdrawal != null && Math.abs(withdrawal.x) < 1e-8 && Math.abs(withdrawal.y - 1) < 1e-8, "source withdrawal direction rotates with the feature");
		var revised = ToolingCodec.snapshot(p.document), e = p.envelope;
		revised.input.revision++;
		revised.input.features.push({id:"extra-pocket",kind:materia.casting.CastingFeature.CastingFeatureKind.Lightening,owner:null,geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(10,10,8)),pose:materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(revised.setup.sourceToMold),
				materia.assembly.AssemblyFrames.translation(e.x + 20, e.y + 20, e.z + e.height - 8))},openings:[materia.casting.CastingPatterns.face("body.z-min")],draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,adjustable:[CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height],derivedFrom:null});
		revised.setup.features.push({
			featureId: "extra-pocket",
			strategy: materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
			withdrawal: {x: 0, y: 0, z: 1},
			priority: 0
		});
		revised.setup.sourceFingerprint = ToolingCodec.inputFingerprint(revised.input);
		var newPlan = CastingMoldPlanner.plan(revised, p.options),
			newMetrics = MoldOrientationStudy.metrics(newPlan, FabricationTests.spec());
		check(newMetrics.formers > metrics.formers && newMetrics.mixMassKg < metrics.mixMassKg,
			"adding a real source pocket changes former effort and casting mass");
		rejects(() -> MoldOrientationStudy.run([a, a], FabricationTests.spec()), "Duplicate orientation candidate");
		var revisedCandidate:MoldOrientationCandidate = {
			id: "revision-two",
			document: revised,
			options: p.options,
			process: s
		};
		rejects(() -> MoldOrientationStudy.run([a, revisedCandidate], FabricationTests.spec()), "same source revision");
		var stale = MoldOrientationStudy.run([a], FabricationTests.spec(), plan -> {
			return {
				planFingerprint: "stale",
				assemblySteps: 0,
				demoldingSteps: 0,
				assemblyBlockingEdges: 0,
				demoldingBlockingEdges: 0,
				validation: [
					materia.tooling.process.FillAnalysis.check("orientation.geometry", [p.document.input.id],
						materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass, "fixture", [])
				]
			};
		}).rankedCandidates[0];
		check(!stale.geometryChecked && stale.validation.filter(v -> v.rule == "orientation.geometry" && v.status == "fail").length == 1,
			"stale CAD evidence cannot mark a candidate checked");
		var checked = MoldOrientationStudy.run([a], FabricationTests.spec(), plan -> {
			return {
				planFingerprint: materia.tooling.casting.CastingMoldCodec.fingerprint(plan),
				assemblySteps: 1,
				demoldingSteps: 1,
				assemblyBlockingEdges: 0,
				demoldingBlockingEdges: 0,
				validation: plan.validation.concat([
					for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"])
						materia.tooling.process.FillAnalysis.check("tooling.cad." + name, [plan.document.input.id],
							materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass, "CAD test evidence", [])
				])
			};
		}).rankedCandidates[0];
		check(checked.geometryChecked, "own four CAD records establish geometry despite pending planner checks");
		var badProcess = ProcessSpecCodec.snapshot(s);
		badProcess.sections = [
			{
				id: "bad-source",
				sourceIds: ["missing-source"],
				spanIds: [],
				geometryFingerprint: null,
				thicknessMm: 20,
				left: boundary,
				right: boundary,
				evidence: []
			}
		];
		var badCandidate:MoldOrientationCandidate = {
			id: "bad-source",
			document: p.document,
			options: p.options,
			process: badProcess
		};
		check(MoldOrientationStudy.run([badCandidate], FabricationTests.spec())
			.rankedCandidates[0].validation.filter(v -> v.rule == "orientation.process" && v.status == "fail").length == 1,
			"candidate analysis failure is recorded without aborting study");
		Sys.println('OrientationSectionTests: $assertions assertions passed');
	}
}
