package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.CastHandling;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.export.FabricationSpec;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingFabrication.FabricationPlan;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.CureAnalysis;

typedef MoldOrientationCandidate = {
	var id:String;
	var document:ToolingDocument;
	var options:CastingMoldOptions;
	var process:CastingProcessSpec;
}

typedef MoldEffortMetrics = {
	var panels:Int;
	var uniquePanels:Int;
	var sheetCount:Int;

	/** Bounding blank area / supplied sheet area, not contour nesting yield. */
	var blankSheetUtilization:Float;

	var formers:Int;
	var epsCores:Int;
	var sacrificialEPSVolumeMm3:Float;
	var logicalHardwareStacks:Int;
	var castingMassKg:Float;
	var mixMassKg:Float;
	var openChangeRequests:Int;
	var requiresFlip:Bool;
}

typedef OrientationGeometryAssessment = {
	var planFingerprint:String;
	var assemblySteps:Int;
	var demoldingSteps:Int;
	var assemblyBlockingEdges:Int;
	var demoldingBlockingEdges:Int;
	var validation:Array<ToolingValidation>;
}

typedef MoldOrientationResult = {
	var id:String;
	var documentFingerprint:String;
	var planFingerprint:Null<String>;
	var planningSucceeded:Bool;
	var geometryChecked:Bool;
	var planningFailure:Null<String>;
	var sequence:Null<OrientationGeometryAssessment>;
	var fill:FillReport;
	var cure:CureReport;
	var sections:OrthogonalSections.OrthogonalSectionReport;
	var handling:HandlingPlan;
	var effort:Null<MoldEffortMetrics>;
	var changeRequests:Array<DesignChangeRequest>;
	var validation:Array<ToolingValidation>;
}

typedef MoldOrientationReport = {
	var version:Int;
	var sourceFingerprint:String;
	var rankingRule:String;
	var rankedCandidates:Array<MoldOrientationResult>;

	/** Review preference only, never a fabrication approval or an automatic setup change. */
	var preferredReviewCandidateId:Null<String>;
}

/** Explicit candidates, real planner, shared process rules. CAD checks are injected
 * by MachineKit and never replaced by a second 2D/removal rule set.
 */
class MoldOrientationStudy {
	/** Directions follow the source feature; setup coordinates and fill height are regenerated. */
	public static function reorient(document:ToolingDocument, id:String, pose:materia.assembly.AssemblyRecord.AssemblyFrame):ToolingDocument {
		materia.tooling.ToolingIds.validate(id);
		var d = ToolingCodec.snapshot(document);
		var relative = materia.assembly.AssemblyFrames.compose(pose, materia.assembly.AssemblyFrames.inverse(d.setup.sourceToMold));
		for (feature in d.setup.features)
			if (feature.withdrawal != null) {
				var v = feature.withdrawal;
				feature.withdrawal = materia.assembly.AssemblyFrames.transformVector(relative, v.x, v.y, v.z);
			}
		d.setup.id = id;
		d.setup.sourceToMold = materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.identity(), pose);
		var envelope = CastingMoldPlanner.envelope(d);
		d.setup.sourceToMold.z -= envelope.z;
		d.setup.fillHeightMm = envelope.height;
		return ToolingCodec.snapshot(d);
	}

	public static function metrics(p:CastingMoldPlan, fabrication:FabricationSpec):MoldEffortMetrics {
		return metricsFrom(p, ToolingFabrication.plan(p, fabrication));
	}

	static function metricsFrom(p:CastingMoldPlan, f:FabricationPlan):MoldEffortMetrics {
		var panels = p.panels.members.concat(p.supportPanels),
			unique:Array<String> = [];
		var blanks = 0.0, sheets = 0, eps = 0.0;
		for (m in panels) {
			var panel = m.panel;
			var key = ToolingCodec.canonical({
				stock: panel.stock,
				outline: PanelContours.outside(panel),
				operations: [
					for (op in panel.operations)
						{
							kind: op.kind,
							contour: op.contour,
							depth: op.depth,
							through: op.through,
							side: op.side,
							toolDiameter: op.toolDiameter,
							accessVia: op.accessVia
						}
				]
			});
			if (unique.indexOf(key) < 0)
				unique.push(key);
		}
		// Exclude trial coupons from production effort, retain their layouts separately.
		var production = f.panelRequirements.filter(r -> panels.filter(m -> m.panel.id == r.partId).length > 0);
		for (r in production)
			blanks += r.blankWidth * r.blankHeight;
		var layout = ToolingFabrication.layout(production, p.document.recipe.panel.width, p.document.recipe.panel.height, f.spec.blankKerfMm);
		for (placement in layout)
			sheets = Std.int(Math.max(sheets, placement.sheet));
		for (c in p.cores)
			eps += BoxVolumes.volume(c.foamLayers);
		var mix = BoxVolumes.subtract([p.envelope], [
			for (c in p.document.input.features)
				CastingMoldPlanner.featureBox(p.document, c)
		]);
		return {
			panels: panels.length,
			uniquePanels: unique.length,
			sheetCount: sheets,
			blankSheetUtilization: sheets == 0 ? 0 : blanks / (sheets * p.document.recipe.panel.width * p.document.recipe.panel.height),
			formers: p.formers.length,
			epsCores: p.cores.length,
			sacrificialEPSVolumeMm3: eps,
			logicalHardwareStacks: p.hardware.length,
			castingMassKg: p.handling.castingMassKg,
			mixMassKg: BoxVolumes.volume(mix) * p.document.recipe.mixDensityKgM3 * 1e-9,
			openChangeRequests: p.changeRequests.length,
			requiresFlip: p.handling.requiresFlip
		};
	}

	public static function count(r:MoldOrientationResult, status:ToolingCheckStatus):Int
		return r.validation.filter(v -> v.status == status).length;

	public static function compare(a:MoldOrientationResult, b:MoldOrientationResult):Int {
		if (a.planningSucceeded != b.planningSucceeded)
			return a.planningSucceeded ? -1 : 1;
		if (a.geometryChecked != b.geometryChecked)
			return a.geometryChecked ? -1 : 1;
		for (status in [ToolingCheckStatus.Fail, ToolingCheckStatus.Unsupported]) {
			var order = Reflect.compare(count(a, status), count(b, status));
			if (order != 0)
				return order;
		}
		var changes = Reflect.compare(a.changeRequests.length, b.changeRequests.length);
		if (changes != 0)
			return changes;
		var pending = Reflect.compare(count(a, ToolingCheckStatus.Pending), count(b, ToolingCheckStatus.Pending));
		if (pending != 0)
			return pending;
		return Reflect.compare(a.id, b.id);
	}

	public static function run(candidates:Array<MoldOrientationCandidate>, fabrication:FabricationSpec,
			?geometry:CastingMoldPlan->OrientationGeometryAssessment):MoldOrientationReport {
		if (candidates.length == 0 || candidates.length > 24)
			throw "Invalid orientation candidate count";
		var results:Array<MoldOrientationResult> = [],
			seen = new Map<String, Bool>(),
			source:Null<String> = null;
		for (candidate in candidates) {
			materia.tooling.ToolingIds.validate(candidate.id);
			if (seen.exists(candidate.id))
				throw "Duplicate orientation candidate";
			seen.set(candidate.id, true);
			var d = ToolingCodec.snapshot(candidate.document),
				fingerprint = ToolingCodec.documentFingerprint(d);
			var input = ToolingCodec.inputFingerprint(d.input);
			if (source != null && source != input)
				throw "Orientation candidates must use the same source revision";
			source = input;
			var spec = ProcessSpecCodec.snapshot(candidate.process);
			CastingProcessAnalysis.validateSectionSources(d, spec);
			if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(d.recipe))
				throw "Stale orientation process recipe";
			var envelope = CastingMoldPlanner.envelope(d),
				features = d.input.features.copy();
			features.sort((a, b) -> Reflect.compare(a.id, b.id));
			var boundaries:Array<FillBoundary> = [for (f in features) {id: f.id, box: CastingMoldPlanner.featureBox(d, f)}];
			var resin = BoxVolumes.subtract([envelope], [for (b in boundaries) b.box]);
			var cores:Array<FillCore> = [];
			for (strategy in d.setup.features)
				if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS) {
					var feature = features.filter(f -> f.id == strategy.featureId)[0],
						clipped = BoxVolumes.intersection(envelope, CastingMoldPlanner.featureBox(d, feature));
					if (clipped != null)
						cores.push({id: materia.tooling.ToolingIds.derive("core-", feature.id), boxes: [clipped]});
				}
			var processFingerprint = haxe.crypto.Sha256.encode(ToolingCodec.canonical({document: fingerprint, spec: spec, gravity: fabrication.gravityMps2}));
			var fill = FillAnalysis.analyze(d.input.id, fingerprint, processFingerprint, resin, boundaries, cores, d.setup.fillHeightMm,
				d.recipe.mixDensityKgM3, fabrication.gravityMps2, spec.pourRateMm3PerSecond, spec.maximumAirEscapeWidthMm);
			var cure = CureAnalysis.analyze(d.input.id, fingerprint, processFingerprint, spec, d.recipe.mixDensityKgM3, d.recipe.core.maximumTemperatureC,
				cores.length > 0);
			var sections = OrthogonalSections.analyze(d.input.id, fingerprint, resin, boundaries);
			var result:MoldOrientationResult = {
				id: candidate.id,
				documentFingerprint: fingerprint,
				planFingerprint: null,
				planningSucceeded: false,
				geometryChecked: false,
				planningFailure: null,
				sequence: null,
				fill: fill,
				cure: cure,
				sections: sections,
				handling: CastHandling.plan(d, candidate.options, envelope),
				effort: null,
				changeRequests: [],
				validation: fill.validation.concat(cure.validation).concat(sections.validation)
			};
			var plan:Null<CastingMoldPlan> = null;
			try
				plan = CastingMoldPlanner.plan(d, candidate.options)
			catch (error:Dynamic)
				result.planningFailure = Std.string(error);
			if (plan == null)
				result.validation.push(FillAnalysis.check("orientation.planner", [d.input.id], ToolingCheckStatus.Fail,
					"Mold planning failed: " + result.planningFailure, [fingerprint]));
			else {
				result.planningSucceeded = true;
				result.planFingerprint = CastingMoldCodec.fingerprint(plan);
				result.changeRequests = plan.changeRequests.copy();
				var fabricated:Null<FabricationPlan> = null;
				try
					fabricated = ToolingFabrication.plan(plan, fabrication)
				catch (error:Dynamic)
					result.validation.push(FillAnalysis.check("orientation.fabrication", [d.input.id], ToolingCheckStatus.Fail,
						"Fabrication planning failed: " + Std.string(error), [result.planFingerprint]));
				if (fabricated != null) {
					result.effort = metricsFrom(plan, fabricated);
					result.validation = result.validation.concat(fabricated.validation);
				} else
					result.validation = result.validation.concat(plan.validation);
				if (geometry == null)
					result.validation.push(FillAnalysis.check("orientation.geometry", [d.input.id], ToolingCheckStatus.Pending,
						"Exact reconstruction, coverage and swept assembly/demolding graphs have not run for this candidate.", []));
				else {
					try {
						var assessment = geometry(plan);
						if (assessment.planFingerprint != result.planFingerprint)
							throw "Stale orientation CAD assessment";
						var checks = assessment.validation;
						result.sequence = assessment;
						if (checks == null || checks.length == 0)
							throw "Empty candidate geometry assessment";
						result.validation = result.validation.concat(checks);
						result.geometryChecked = checks.filter(v -> v.status != ToolingCheckStatus.Pass).length == 0;
					} catch (error:Dynamic)
						result.validation.push(FillAnalysis.check("orientation.geometry", [d.input.id], ToolingCheckStatus.Fail,
							"Candidate CAD/swept-graph assessment failed: " + Std.string(error), [result.planFingerprint]));
				}
			}
			result.validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
			results.push(result);
		}
		results.sort(compare);
		var best = results[0],
			preferred = best.planningSucceeded && best.geometryChecked && count(best, ToolingCheckStatus.Fail) == 0 ? best.id : null;
		return {
			version: 1,
			sourceFingerprint: source,
			rankingRule: "planned, CAD-checked, failures, unsupported, source requests, pending, candidate ID",
			rankedCandidates: results,
			preferredReviewCandidateId: preferred
		};
	}
}
