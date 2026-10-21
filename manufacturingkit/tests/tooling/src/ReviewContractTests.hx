import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.CastingMoldOptionsCodec;
import materia.casting.BoxAlgebra;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingLoads;
import materia.tooling.process.CastingLevelOne;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CureAnalysis;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.JunctionThickness;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.process.ThermalCoverage;
import materia.tooling.process.ThermalFaces;

class ReviewContractTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function rejects(fn:Void->Void, expected:String):Void {
		var message = "";
		try
			fn()
		catch (error:Dynamic)
			message = Std.string(error);
		check(message.indexOf(expected) >= 0, 'Expected "$expected", got "$message"');
	}

	static function boundary(kind:ThermalBoundaryKind):ThermalBoundary
		return {
			kind: kind,
			temperatureC: kind == Insulated ? null : 20,
			heatTransferWPerM2K: kind == Convection ? 10 : null,
			evidence: ["test contact"]
		};

	public static function run(p:CastingMoldPlan):Void {
		var o = CastingMoldOptionsCodec.snapshot(p.options);
		var source=ToolingCodec.snapshot(p.document);
		source.input.inserts[0].finish=materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting;
		source.setup.sourceFingerprint=ToolingCodec.inputFingerprint(source.input);
		o.precision[0].masterReference = null;
		check(CastingMoldPlanner.plan(source, o)
			.validation.filter(v -> v.rule == "tooling.precision.replication." + o.precision[0].insertId && v.status == "pending")
			.length == 1,
			"missing replication master has its explicit validation rule");
		var missing=CastingMoldPlanner.plan(source,o);
		missing.validation=missing.validation.filter(v -> v.rule!="tooling.precision.replication."+o.precision[0].insertId);
		missing.fabrication=ToolingReadiness.evaluate(missing.validation);
		rejects(() -> CastingMoldPlanner.validate(missing),"Replication requirement differs from source interfaces and process choices");
		var oldOptions:Dynamic=haxe.Json.parse(ToolingCodec.canonical(o));
		var oldPrecision:Array<Dynamic> = cast Reflect.field(oldOptions,"precision");
		Reflect.setField(oldPrecision[0],"allowance",null);
		rejects(() -> CastingMoldOptionsCodec.read(oldOptions),"Invalid mold process record");
		Reflect.deleteField(oldPrecision[0],"allowance");
		Reflect.setField(oldPrecision[0],"route","replicated");
		rejects(() -> CastingMoldOptionsCodec.read(oldOptions),"Invalid mold process record");
		Reflect.deleteField(oldPrecision[0],"route");
		Reflect.deleteField(oldPrecision[0],"jigReference");
		rejects(() -> CastingMoldOptionsCodec.read(oldOptions),"Invalid mold process record");
		o.precision[0].jigReference="synthetic-jig";
		check(CastingMoldOptionsCodec.snapshot(o).precision[0].jigReference=="synthetic-jig",
			"jig reference is an explicit process choice");
		o.precision[0].masterReference="synthetic-master";o.precision[0].releaseSpec="synthetic-release";
		check(CastingMoldPlanner.plan(source,o).validation.filter(v ->
			v.rule=="tooling.precision.replication."+o.precision[0].insertId).length==0,
			"identified master and release clear only the replication prerequisites");
		source.input.inserts[0].finish=materia.casting.InsertSpec.InsertFinish.AsSupplied;
		source.setup.sourceFingerprint=ToolingCodec.inputFingerprint(source.input);
		o.precision[0].masterReference=null;o.precision[0].releaseSpec=null;
		check(CastingMoldPlanner.plan(source,o).validation.filter(v ->
			v.rule=="tooling.precision.replication."+o.precision[0].insertId).length==0,
			"process options cannot introduce replication into an as-supplied source");
		var sourceInsert=source.input.inserts[0];
		sourceInsert.interfaces=[{id:"replicated-plane",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(
			materia.assembly.AssemblyFrames.translation(0,0,21)),route:materia.casting.InterfaceRequirement.InterfaceRoute.Replicated,
			allowanceMm:1,finishingProcessErrorMm:null,permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]}];
		check(materia.tooling.casting.InterfaceQualification.replicationRequirement(sourceInsert,o.precision[0])!=null,
			"explicit source interfaces require replication independently of the aggregate finish label");
		var raw = ToolingCodec.encode(p.document);
		check(raw.indexOf('"label"') >= 0 && raw.indexOf('"sourceRole"') < 0, "source labels are separate from required typed roles");
		var changed = ToolingCodec.snapshot(p.document);
		changed.recipe.designReview.minimumInsertEdgeMm = 5;
		var constrained = CastingLevelOne.analyze(changed).proposals.filter(v -> v.rule == "process.level1.insert-edge.seat");
		check(constrained.length == 1 && constrained[0].proposedValue == null,
			"width proposals preserve explicitly declared body openings");
		// An independently authored inset seat permits width changes without changing openings.
		changed.input.inserts[0].pose.x = 78;
		var recess = changed.input.features.filter(feature -> feature.owner == "seat")[0];
		recess.geometry.pose.x = 78;
		recess.openings = recess.openings.filter(opening -> opening.target != "body.x-max");
		changed.setup.sourceFingerprint = ToolingCodec.inputFingerprint(changed.input);
		var review = CastingLevelOne.analyze(changed);
		var proposals = review.proposals.filter(v -> v.rule == "process.level1.insert-edge.seat");
		check(proposals.length == 1 && proposals[0].sourceParameterId == "input.inserts.seat.body.solid.width" && proposals[0].proposedValue == 34,
			"source-only edge review supplies a mapped width/value proposal");
		check(review.metrics.minimumInsertEdgeMm == 2
			&& review.metrics.minimumCornerRadiusMm == 0, "level-1 metrics retain actual source geometry");
		var accepted = CastingLevelOne.apply(changed, proposals[0]);
		var replanned = CastingMoldPlanner.plan(accepted, p.options);
		check(InsertBodies.box(replanned.document.input.inserts[0].body, "seat").width == 34 && replanned.sourceFingerprint != p.sourceFingerprint,
			"accepted source proposal updates linked recess and successfully replans");
		check(CastingLevelOne.analyze(accepted).validation.filter(v -> v.rule == "process.level1.insert-edge.seat"
			&& v.status == "pass").length == 1,
			"accepted width change clears the actual source edge rule");
		rejects(() -> CastingLevelOne.apply(accepted, proposals[0]), "stale source proposal");
		var limits = ToolingCodec.snapshot(changed);
		limits.recipe.designReview.minimumCornerRadiusMm = 2;
		limits.recipe.designReview.minimumInsertCoverMm = 20;
		limits.recipe.designReview.minimumFixingAccessMm = 20;
		limits.recipe.designReview.minimumLiftingPoints = 1;
		limits.recipe.designReview.removalClearanceMm = 0;
		limits.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer;
		limits.setup.features[0].withdrawal = {x: 0, y: 0, z: -1};
		var levelOne = CastingLevelOne.analyze(limits);
		for (rule in [
			"corner-radius.pocket",
			"insert-cover.seat",
			"insert-edge.seat",
			"fixing-access.seat",
			"lifting-points",
			"removal-direction.pocket"
		])
			check(levelOne.validation.filter(v -> v.rule == "process.level1." + rule && v.status == "fail").length == 1,
				"source-only declared-threshold rule is exercised: " + rule);
		var s = FabricationTests.spec();
		s.plates = [
			{
				panelId: p.panels.members[0].panel.id,
				spanXmm: 100,
				spanYmm: 100,
				youngPa: 3e9,
				poisson: 0.3,
				allowableStressPa: 1e7,
				maximumDeflectionMm: 1,
				evidence: ["test"]
			}
		];
		rejects(() -> ToolingLoads.report(p, s), "without structural model");
		var invalid = FillAnalysis.check("test.rule", [p.document.input.id], materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail, "test", []);
		invalid.severity = cast "unknown";
		rejects(() -> ToolingReadiness.evaluate([invalid]), "severity/status consistency");
		invalid.severity = materia.tooling.ToolingDiagnostic.ToolingSeverity.Info;
		rejects(() -> ToolingReadiness.evaluate([invalid]), "severity/status consistency");
		var lower = ToolingCodec.snapshot(p.document);
		lower.setup.fillHeightMm = p.envelope.z + p.envelope.height / 2;
		check(CastingMoldPlanner.resinAtFill(lower, p.envelope).filter(b -> b.z + b.height > lower.setup.fillHeightMm + 1e-7).length == 0,
			"neutral datum/resin mask excludes material above fill height");
		var bridges = CastingMoldCodec.decode(CastingMoldCodec.encode(p));
		var wider = CastingMoldCodec.decode(CastingMoldCodec.encode(p)).cores[0];
		wider.id = "wide-core";
		wider.carrierId = "wide-carrier";
		var cy = wider.cavity.y + wider.cavity.depth / 2;
		wider.cavity.depth = 200;
		wider.cavity.y = cy - 100;
		bridges.cores.push(wider);
		materia.tooling.casting.CoreSupports.sharedBridges(bridges.cavity, bridges.document.recipe, bridges.cores, [], []);
		var foam = bridges.cores[0].foamLayers[0];
		check(Math.abs(bridges.cores[0].bridgeBondAreaMm2 - foam.width * foam.depth) < 1e-7,
			"a wider row beam never credits bond area beyond the smaller foam pad");
		var spec = ProcessTests.spec();
		spec.solver.durationSeconds = 1;
		spec.solver.maximumExtensionSeconds = 2000;
		var extended = CureAnalysis.section(spec.sections[0], spec, 2400);
		check(extended.simulatedSeconds > 1 && extended.reactionComplete && !extended.extensionCapReached,
			"cure continues past nominal duration until residual heat meets tolerance");
		spec.solver.maximumExtensionSeconds = 2;
		var capped = CureAnalysis.analyze("body", "source", "process", spec, 2400, 100, true);
		check(capped.sections[0].extensionCapReached
			&& capped.validation.filter(v -> v.rule == "process.cure.section.wall" && v.status == "pending").length == 1,
			"extension cap leaves incomplete reaction Pending");
		var box:materia.tooling.casting.MoldPlan.MoldBox = {
			x: 0,
			y: 0,
			z: 0,
			width: 100,
			depth: 60,
			height: 20
		};
		var geometry = OrthogonalSections.analyze("body", "source", [box], []);
		var cold = geometry.junction.faces.filter(face -> face.axis == Z && face.planeMm == 0);
		var oneSided = JunctionThickness.analyze("body", "source", [box], [], {diameterToleranceMm: 0.01, maximumCells: 50000}, cold);
		check(geometry.junction.lowerDiameterMm <= 20
			&& oneSided.lowerDiameterMm <= 40 + 1e-7
			&& oneSided.upperDiameterMm >= 40 - 1e-7
			&& oneSided.converged
			&& oneSided.lowerDiameterMm > 39.99,
			"one cooled face doubles the effective slab thickness");
		var rigid = ToolingCodec.snapshot(p.document);
		for (strategy in rigid.setup.features)
			if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS) {
				strategy.strategy = materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer;
				strategy.withdrawal = {x: 0, y: 0, z: 1};
			}
		var noCore = CastingMoldPlanner.plan(rigid, p.options);
		s = FabricationTests.spec();
		s.solvent = StyreneBearing;
		s.compatibilityEvidence = ["test resin"];
		check(noCore.cores.length == 0
			&& ToolingFabrication.plan(noCore, s).validation.filter(v -> v.rule == "fabrication.materials.compatibility"
				&& v.status == "fail").length == 1,
			"unknown adhesive plus styrene fails even without cores");
		spec = ProcessTests.spec();
		spec.recipeFingerprint = ToolingCodec.recipeFingerprint(rigid.recipe);
		spec.sections = [];
		var unknown = CastingProcessAnalysis.analyze(noCore, spec, 9.80665);
		check(unknown.cure.hasEPS, "missing fabrication record conservatively includes EPS");
		unknown.cure.hasEPS = false;
		rejects(() -> ToolingFabrication.plan(noCore, s, unknown.cure), "Stale fabrication cure report");
		var sharedSpec = ProcessTests.spec();
		sharedSpec.recipeFingerprint = ToolingCodec.recipeFingerprint(p.document.recipe);
		sharedSpec.sections = [];
		var shared = CastingProcessAnalysis.analyze(p, sharedSpec, 9.80665, FabricationTests.spec());
		if (shared.sections==null) throw "Box fixture requires exact orthogonal sections";
		var sharedSections:materia.tooling.process.OrthogonalSections.OrthogonalSectionReport=cast shared.sections;
		var eps = [
			for (strategy in p.document.setup.features) if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS)
				strategy.featureId
		];
		sharedSpec.sections = [
			for (span in sharedSections.spans)
				if (span.axis == Z) OrthogonalSections.cureSection(span,
					boundary(span.lowerSourceIds.filter(id -> eps.indexOf(id) >= 0).length > 0 ? Insulated : Convection),
					boundary(span.upperSourceIds.filter(id -> eps.indexOf(id) >= 0).length > 0
						|| Math.abs(span.endMm - p.document.setup.fillHeightMm) < 1e-7 ? Insulated : Convection),
					["test source-owned boundaries"], sharedSections.geometryFingerprint)
		];
		var covered = CastingProcessAnalysis.analyze(p, sharedSpec, 9.80665, FabricationTests.spec());
		if (covered.thermalCoverage==null) throw "Box fixture requires thermal coverage";
		var coverage:materia.tooling.process.ThermalCoverage.ThermalCoverageReport=cast covered.thermalCoverage;
		check(coverage.validation.filter(v -> v.rule == "process.cure.geometric-coverage"
			&& v.status == "pass").length == 1,
			"source-owned disjoint slabs cover the actual casting");
		check(covered.cure.validation.filter(v -> v.rule == "process.cure.section-coverage" && v.status == "pass").length == 1,
			"CureReport and report-level geometric coverage agree on Pass");
		var duplicate = sharedSections.spans.filter(span -> span.axis == X)[0];
		sharedSpec.sections.push(OrthogonalSections.cureSection(duplicate, boundary(Convection), boundary(Convection), [],
			sharedSections.geometryFingerprint));
		check(ThermalCoverage.analyze(p.document.input.id, sharedSections, sharedSpec, "process")
			.validation.filter(v -> v.rule == "process.cure.geometric-coverage" && v.status == "fail")
			.length == 1,
			"different-axis sections cannot cover the same volume twice");
		sharedSpec.sections.pop();
		var epsSection = sharedSpec.sections.filter(section -> section.left.kind == Insulated || section.right.kind == Insulated)[0];
		if (epsSection.left.kind == Insulated)
			epsSection.left = boundary(Convection);
		else
			epsSection.right = boundary(Convection);
		check(ThermalCoverage.analyze(p.document.input.id, sharedSections, sharedSpec, "process", p.document)
			.validation.filter(v -> v.rule == "process.cure.geometric-coverage" && v.status == "fail")
			.length == 1,
			"a cooling boundary cannot be assigned to an EPS or undeclared open face");
		Sys.println('ReviewContractTests: $assertions assertions passed');
	}
}
