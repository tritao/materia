import materia.casting.CastingProfiles as Profiles;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CastabilityAnalysis;
import materia.tooling.process.ProcessSpecCodec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.OrthogonalSections;

class CastabilityTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function rejects(fn:Void->Void, expected:String):Void {
		var error = "";
		try
			fn()
		catch (e:Dynamic)
			error = Std.string(e);
		check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
	}

	static function box(x:Float, z:Float, width:Float, depth:Float, height:Float):MoldBox
		return {
			x: x,
			y: 0,
			z: z,
			width: width,
			depth: depth,
			height: height
		};

	static function status(r:CastabilityReport, rule:String):String {
		var rows = r.validation.filter(v -> v.rule == rule);
		check(rows.length == 1, "one finding per rule: " + rule);
		return rows[0].status;
	}

	static function liftingLocations():Void {
		var d=ToolingTests.fixture();d.recipe.designReview.minimumLiftingPoints=4;
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var report=materia.tooling.process.CastingLevelOne.analyze(d);
		var locations=report.proposals.filter(proposal -> proposal.action==DeclareLiftingPoint);
		check(locations.length==4,"missing lifting connections get a complete CoM-based review pattern");
		var envelope=materia.tooling.casting.CastingMoldPlanner.envelope(d);
		var moments=materia.tooling.casting.CastHandling.moments(d,envelope),x=0.0,y=0.0;
		for (location in locations) {
			if (location.point==null) throw "Missing lifting candidate point";
			var point:{x:Float,y:Float,z:Float}=cast location.point;
			check(Math.isFinite(point.x+point.y+point.z)
				&& location.instruction.indexOf("nominal casting mass")>=0,"each candidate names its source position and nominal mass basis");
			x+=point.x;y+=point.y;
		}
		check(Math.abs(x/locations.length-moments.centerOfMassMoldMm.x)<1e-8
			&& Math.abs(y/locations.length-moments.centerOfMassMoldMm.y)<1e-8,
			"candidate pattern surrounds the density-weighted casting CoM");
		var original=x/locations.length;d.recipe.steelDensityKgM3*=2;
		var shifted=materia.tooling.process.CastingLevelOne.analyze(d).proposals.filter(proposal -> proposal.action==DeclareLiftingPoint);
		var moved=0.0;
		for (location in shifted) {
			if (location.point==null) throw "Missing shifted lifting candidate";
			var point:{x:Float,y:Float,z:Float}=cast location.point;
			moved+=point.x;
		}
		check(Math.abs(moved/shifted.length-original)>1e-3,"a changed steel mass shifts the lifting pattern, not just its label");
		rejects(() -> materia.tooling.process.CastingLevelOne.apply(d,shifted[0]),"Unmapped or stale source proposal");
	}

	static function authoredProfileCorners():Void {
		var semicircle = [Profiles.arc(0, 0, 40, -Math.PI / 2, Math.PI),
			Profiles.line({x: 0, y: 40}, {x: 0, y: -40})];
		var cases = [
			{solid: Cut(materia.casting.CastingSolid.Box(100, 80, 61)), rounding: 6.0, measured: 6.0},
			{solid: Prism(Profiles.roundedRectangle(-50, -40, 100, 80, 6), 61), rounding: 0.0, measured: 6.0},
			{solid: Prism(Profiles.roundedRectangle(-50, -40, 100, 80, 2), 61), rounding: 0.0, measured: 2.0},
			{solid: Prism(Profiles.rectangle(-50, -40, 100, 80), 61), rounding: 0.0, measured: 0.0},
			{solid: Cut(materia.casting.CastingSolid.Cylinder(20, 61)), rounding: 0.0, measured: 10.0},
			{solid: Prism(semicircle, 61), rounding: 0.0, measured: 0.0}
		];
		for (sample in cases) {
			var document = ToolingTests.fixture();
			document.recipe.designReview.minimumCornerRadiusMm = 4;
			var feature = document.input.features.filter(feature -> feature.id == "pocket")[0];
			feature.geometry.solid = sample.solid;
			feature.cornerRadius = sample.rounding;
			document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
			var report = materia.tooling.process.CastingLevelOne.analyze(document);
			var radius = report.metrics.minimumCornerRadiusMm;
			check(radius != null && Math.abs(radius - sample.measured) < 1e-7,
				"source screening measures authored line/arc corners rather than the optional implicit-filleting parameter");
			var rows = report.validation.filter(row -> row.rule == "process.level1.corner-radius.pocket");
			check(rows.length == 1 && rows[0].status == (sample.measured >= 4 ? "pass" : "fail"),
				"the existing corner-radius rule tests actual curvature and detects sharp arc-to-line joints");
		}
	}

	static function nonBoxReports():Void {
		var document = ToolingTests.fixture();
		document.recipe.designReview.minimumLiftingPoints = 4;
		document.recipe.designReview.minimumInsertEdgeMm = 0;
		document.recipe.designReview.minimumInsertCoverMm = 1;
		document.recipe.designReview.minimumFixingAccessMm = 1;
		var baseline = materia.tooling.process.CastingLevelOne.analyze(document);
		var feature = document.input.features.filter(feature -> feature.id == "pocket")[0];
		feature.cornerRadius = 4;
		feature.derivedFrom = "authored-pocket";
		document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
		var before = ToolingCodec.canonical(document);
		var spec = ProcessTests.spec();
		spec.recipeFingerprint = ToolingCodec.recipeFingerprint(document.recipe);
		spec.castability.maximumAggregateSizeMm = 4;
		spec.castability.minimumPassageAggregateRatio = 3;
		spec.castability.maximumSectionThicknessMm = 30;
		var source = ToolingCodec.documentFingerprint(document), process = "partial-process";
		var fill = FillAnalysis.analyze(document.input.id, source, process,
			[box(0, 0, 2, 2, 10), box(2, 0, 2, 2, 2), box(4, 0, 4, 2, 6)],
			[{id: "authored-pocket", box: box(4, 6, 4, 2, 4)}], [], 10, 2400, 10, null, 100);
		var report = CastabilityAnalysis.analyze(document, spec, fill, null, source, process);
		check(report.minimumOpposingFaceSpanMm == null && report.maximumInscribedThicknessMm == null
			&& report.maximumDiameterLowerMm == null && report.maximumDiameterUpperMm == null
			&& report.heatRemovingInterval == null, "missing section geometry supplies no invented scalar measurements");
		for (rule in ["process.castability.minimum-passage", "process.castability.maximum-section"]) {
			check(status(report, rule) == "unsupported", "a declared limit does not qualify missing geometry: " + rule);
			var row = report.validation.filter(row -> row.rule == rule)[0];
			check(row.affectedIds.length == 1 && row.affectedIds[0] == "authored-pocket",
				"unsupported measurements name the authored feature rather than a fragment or body bound");
		}
		check(status(report, "process.level1.lifting-points") == "fail",
			"a non-box cavity cannot hide missing typed lifting connections");
		for (name in ["insert-edge", "insert-cover", "fixing-access"])
			check(status(report, "process.level1." + name + ".seat") == "pass",
				"a curved pocket does not suppress independent exact insert checks: " + name);
		check(report.designMetrics.minimumInsertEdgeMm == baseline.metrics.minimumInsertEdgeMm
			&& report.designMetrics.minimumEmbeddedDepthMm == baseline.metrics.minimumEmbeddedDepthMm
			&& report.designMetrics.minimumFixingAccessLandMm == baseline.metrics.minimumFixingAccessLandMm,
			"independent box-stock measurements are unchanged by an unrelated curved cavity");
		var vent = report.proposals.filter(proposal -> proposal.action == RouteVent);
		check(vent.length == 1 && vent[0].point != null, "partial fill keeps its vent position");
		var ventPoint:{x:Float,y:Float,z:Float} = cast vent[0].point;
		check(vent[0].sourceIds[0] == "authored-pocket" && ventPoint.z == 6,
			"exact fill evidence retains its owned vent proposal without orthogonal sections");
		check(report.validation.filter(row -> row.rule == vent[0].rule && row.status == "fail").length == 1,
			"retaining a vent suggestion never clears the actual trapped-air failure");
		var sourceOnly = CastabilityAnalysis.analyze(document, spec, null, null, source, process);
		check(sourceOnly.sourceFingerprint == source && sourceOnly.processFingerprint == process
			&& sourceOnly.proposals.filter(proposal -> proposal.action == RouteVent).length == 0,
			"an unavailable fill report keeps source checks without inventing pockets or vent locations");
		check(ToolingCodec.canonical(document) == before, "partial geometry screening leaves authored source facts unchanged");
		document.input.features.reverse();
		document.setup.features.reverse();
		check(ToolingCodec.canonical(CastabilityAnalysis.analyze(document, spec, fill, null, source, process))
			== ToolingCodec.canonical(report), "partial reports and proposal IDs do not depend on feature order");
		rejects(() -> CastabilityAnalysis.analyze(document, spec, fill, null, source, "stale"),
			"Mismatched castability fill process");
		var fragments = ToolingCodec.snapshot(document);
		var sibling = materia.casting.CastingFeatureCodec.snapshot(fragments.input.features.filter(feature -> feature.id == "pocket"))[0];
		sibling.id = "pocket-second";
		fragments.input.features.push(sibling);
		fragments.setup.sourceFingerprint = ToolingCodec.inputFingerprint(fragments.input);
		var grouped = materia.tooling.process.CastingLevelOne.analyze(fragments);
		var geometryRows = grouped.validation.filter(row -> row.rule == "process.level1.geometry.authored-pocket");
		check(geometryRows.length == 1 && geometryRows[0].message.indexOf("pocket-second") >= 0
			&& geometryRows[0].message.indexOf("pocket") >= 0,
			"non-box fragments retain their reasons in one finding for the authored parent");
		fragments.input.features.reverse();
		check(ToolingCodec.canonical(materia.tooling.process.CastingLevelOne.analyze(fragments)) == ToolingCodec.canonical(grouped),
			"grouped geometry findings and messages are independent of fragment order");
		var curvedBody = ToolingCodec.snapshot(document);
		curvedBody.input.envelope = materia.casting.CastingEnvelope.build(200, 180, 100, 10);
		curvedBody.setup.sourceFingerprint = ToolingCodec.inputFingerprint(curvedBody.input);
		var sourceChecks = materia.tooling.process.CastingLevelOne.analyze(curvedBody);
		for (name in ["insert-edge", "insert-cover"])
			check(sourceChecks.validation.filter(row -> row.rule == "process.level1." + name + ".seat"
				&& row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length == 1,
				"a chamfered body does not receive a bounding-box insert measurement: " + name);
		check(sourceChecks.metrics.minimumInsertEdgeMm == null && sourceChecks.metrics.minimumEmbeddedDepthMm == null
			&& sourceChecks.metrics.minimumFixingAccessLandMm == baseline.metrics.minimumFixingAccessLandMm,
			"unsupported body measurements stay null while the actual insert's fixing land stays measured");
		check(sourceChecks.validation.filter(row -> row.rule == "process.level1.lifting-points"
			&& row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length == 1,
			"non-rectangular bodies retain their typed lifting-point rule");
		var old:Dynamic = haxe.Json.parse(ProcessSpecCodec.encode(spec));
		Reflect.setField(old, "version", 9);
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(old)), "Unsupported casting process");
		fill.sourceFingerprint = "stale";
		rejects(() -> CastabilityAnalysis.analyze(document, spec, fill, null, source, process),
			"Mismatched castability section/fill source");
	}

	public static function run(p:CastingMoldPlan, raw:CastingProcessSpec):Void {
		liftingLocations();
		authoredProfileCorners();
		nonBoxReports();
		var d = ToolingCodec.snapshot(p.document),
			s = ProcessSpecCodec.snapshot(raw);
		s.recipeFingerprint = ToolingCodec.recipeFingerprint(d.recipe);
		s.sections = [];
		s.castability = {
			maximumAggregateSizeMm: 4,
			minimumPassageAggregateRatio: 3,
			maximumSectionThicknessMm: 30,
			evidence: ["fixture-assumption-not-qualified"]
		};
		s.maximumAirEscapeWidthMm = 100;
		var source = ToolingCodec.documentFingerprint(d),
			owner = d.input.features[0].id;
		function analyze(region:Array<MoldBox>, ?boundaries:Array<FillBoundary>):CastabilityReport {
			if (boundaries == null)
				boundaries = [];
			var fillHeight = 0.0;
			for (b in region)
				fillHeight = Math.max(fillHeight, b.z + b.height);
			var fill = FillAnalysis.analyze(d.input.id, source, "process", region, boundaries, [], fillHeight, 2400, 10, null, s.maximumAirEscapeWidthMm);
			var sections = OrthogonalSections.analyze(d.input.id, source, region, boundaries);
			return CastabilityAnalysis.analyze(d, s, fill, sections, source, "process", p.validation);
		}
		var slender = analyze([box(0, 0, 100, 60, 20)]);
		check(status(slender, "process.castability.minimum-passage") == "pass", "sufficient passage passes");
		check(status(slender, "process.castability.maximum-section") == "pass", "long axes are not thermal thickness");
		check(slender.maximumInscribedThicknessMm == 20, "rectangular thickness uses shortest dimension");
		var thin = analyze([box(0, 0, 100, 60, 8)], [{id: owner, box: box(0, 8, 100, 60, 10)}]);
		check(status(thin, "process.castability.minimum-passage") == "fail", "aggregate-based narrow passage fails");
		var widen = thin.proposals.filter(v -> v.action == WidenPassage);
		check(widen.length == 1 && widen[0].sourceIds.indexOf(owner) >= 0, "narrow passage reports source feature");
		check(widen[0].measuredMm == 8 && widen[0].targetMm == 12 && widen[0].axis == Z, "proposal carries exact dimension and target");
		var proposalRegion:MoldBox = cast widen[0].region;
		check(proposalRegion.height == 8 && widen[0].owner == DesignReview, "source proposal has mold-space location and owner");
		var atLimit = analyze([box(0, 0, 100, 60, 12)]);
		check(status(atLimit, "process.castability.minimum-passage") == "pass", "equality at aggregate limit passes");
		var thick = analyze([box(0, 0, 100, 60, 40)]);
		check(status(thick, "process.castability.maximum-section") == "fail", "thick block fails declared section screen");
		check(thick.proposals.filter(v -> v.action == ReduceSection && v.measuredMm == 40 && v.targetMm == 30).length == 1,
			"thick section has concrete target without modifying source");
		var region = [box(0, 0, 2, 2, 10), box(2, 0, 2, 2, 2), box(4, 0, 4, 2, 6)];
		var boundaries = [{id: owner, box: box(4, 6, 4, 2, 4)}];
		var trapped = analyze(region, boundaries),
			vents = trapped.proposals.filter(v -> v.action == RouteVent);
		check(vents.length == 1 && vents[0].sourceIds.indexOf(owner) >= 0, "trapped air preserves source owner");
		var point:{x:Float, y:Float, z:Float} = cast vents[0].point;
		check(point.z == 6 && vents[0].owner == ToolingReview, "vent is a tooling-routing candidate at exact ceiling");
		check(trapped.validation.filter(v -> v.rule == vents[0].rule && v.status == "fail").length == 1, "vent suggestion never clears trap");
		check(status(trapped, "process.castability.maximum-section") == "pass", "certified junction upper bound clears a large screen limit");
		check(trapped.proposals.filter(v -> v.action == AnalyzeJunction).length == 0, "resolved geometry does not request an unsupported thickness algorithm");
		var before = ToolingCodec.canonical(trapped);
		region.reverse();
		check(ToolingCodec.canonical(analyze(region, boundaries)) == before, "geometry order does not change findings or proposal IDs");
		var vented = analyze(region.concat([box(6, 6, 2, 2, 4)]), boundaries);
		check(vented.proposals.filter(v -> v.action == RouteVent).length == 0, "actual vent clears topology suggestion");
		s.maximumAirEscapeWidthMm = 1;
		var broad = analyze(region, boundaries);
		check(status(broad, "process.fill.air-escape-width") == "fail", "broad ceiling uses real fill rule");
		check(broad.proposals.filter(v -> v.action == ReviewCeiling && v.sourceIds.indexOf(owner) >= 0).length == 1,
			"broad ceiling suggestion retains feature IDs");
		var cross:Array<MoldBox> = [
			{
				x: -5,
				y: -1,
				z: -4,
				width: 10,
				depth: 2,
				height: 8
			},
			{
				x: -1,
				y: -5,
				z: -4,
				width: 2,
				depth: 10,
				height: 8
			}
		];
		var crossSections = OrthogonalSections.analyze(d.input.id, source, cross, []);
		crossSections.junction = materia.tooling.process.JunctionThickness.analyze(d.input.id, source, cross, [],
			{diameterToleranceMm: 1e-8, maximumCells: 20});
		var crossFill = FillAnalysis.analyze(d.input.id, source, "process", cross, [], [], 4, 2400, 10, null, null);
		var interval = crossSections.junction;
		s.castability.maximumSectionThicknessMm = (interval.lowerDiameterMm + interval.upperDiameterMm) / 2;
		var unresolved = CastabilityAnalysis.analyze(d, s, crossFill, crossSections, source, "process");
		check(status(unresolved, "process.castability.maximum-section") == "pending", "straddling thickness interval never accepts its midpoint");
		s.castability.maximumSectionThicknessMm = interval.upperDiameterMm + 1;
		check(status(CastabilityAnalysis.analyze(d, s, crossFill, crossSections, source, "process"), "process.castability.maximum-section") == "pass",
			"safe upper bound can resolve a screen without geometry convergence");
		s.castability.maximumSectionThicknessMm = interval.lowerDiameterMm - 1;
		check(status(CastabilityAnalysis.analyze(d, s, crossFill, crossSections, source, "process"), "process.castability.maximum-section") == "fail",
			"unsafe lower bound rejects a screen without geometry convergence");
		s.castability.maximumAggregateSizeMm = null;
		s.castability.maximumSectionThicknessMm = null;
		s.maximumAirEscapeWidthMm = null;
		var unknown = analyze([box(0, 0, 100, 60, 20)]);
		check(status(unknown, "process.castability.minimum-passage") == "pending", "unknown aggregate never passes");
		check(status(unknown, "process.castability.maximum-section") == "pending", "unknown section limit never passes");
		check(status(unknown, "process.fill.air-escape-width") == "pending", "unknown ceiling rule remains pending");
		for (strategy in d.setup.features) {
			if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS)
				check(unknown.proposals.filter(v -> v.action == QualifyBreakout
					&& v.sourceIds.indexOf(strategy.featureId) >= 0).length == 1,
					"each EPS source has a physical breakout action");
			else if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer) {
				var actual = p.validation.filter(v -> v.rule == "tooling.former.draft." + strategy.featureId);
				for (v in actual)
					check(unknown.validation.filter(row -> ToolingCodec.canonical(v) == ToolingCodec.canonical(row)).length == 1,
						"rigid release reuses exact planner validation");
			}
		}
		var unchanged = ToolingCodec.documentFingerprint(p.document);
		d.setup.features.reverse();
		d.input.features.reverse();
		check(ToolingCodec.canonical(analyze([box(0, 0, 100, 60, 20)])) == ToolingCodec.canonical(unknown),
			"source/setup member order does not change screening or proposal IDs");
		var unsupportedFeature = d.setup.features[0].featureId;
		d.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported;
		var unsupported = analyze([box(0, 0, 100, 60, 20)]);
		check(status(unsupported, "process.castability.release." + unsupportedFeature) == "unsupported", "unsupported strategy remains an explicit blocker");
		check(unsupported.proposals.filter(v -> v.action == SplitOrSacrificial
			&& v.sourceIds.indexOf(unsupportedFeature) >= 0).length == 1,
			"unsupported strategy gets a tooling action with its source ID");
		d = ToolingCodec.snapshot(p.document);
		check(ToolingCodec.documentFingerprint(p.document) == unchanged, "screening and suggestions preserve the original source/setup");
		for (proposal in unknown.proposals)
			check(unknown.validation.filter(v -> v.rule == proposal.rule).length == 1, "proposal references real finding");
		var fill = FillAnalysis.analyze(d.input.id, source, "process", [box(0, 0, 100, 60, 20)], [], [], 20, 2400, 10, null, null);
		var sections = OrthogonalSections.analyze(d.input.id, "stale", [box(0, 0, 100, 60, 20)], []);
		rejects(() -> CastabilityAnalysis.analyze(d, s, fill, sections, source, "process"), "Mismatched castability");
		sections.sourceFingerprint = source;
		rejects(() -> CastabilityAnalysis.analyze(d, s, fill, sections, source, "stale-process"), "Mismatched castability fill process");
		var rawSpec:Dynamic = haxe.Json.parse(ProcessSpecCodec.encode(s));
		rawSpec.spec.castability.minimumPassageAggregateRatio = 0.5;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(rawSpec)), "Invalid casting process number");
		rawSpec = haxe.Json.parse(ProcessSpecCodec.encode(s));
		Reflect.deleteField(rawSpec.spec.castability, "maximumSectionThicknessMm");
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(rawSpec)), "Missing casting process field");
		s.recipeFingerprint = haxe.crypto.Sha256.encode("stale");
		rejects(() -> analyze([box(0, 0, 100, 60, 20)]), "Stale castability recipe");
		Sys.println('CastabilityTests: $assertions assertions passed');
	}
}
