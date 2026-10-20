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
import materia.tooling.process.CastingProcessAnalysis.CastingProcessReport;
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

typedef OrientationMetricTuple = {
	var failCount:Int;
	var openChangeRequests:Int;
	var trappedVolumeMm3:Float;
	var worstCureMarginC:Null<Float>;
	var requiresFlip:Bool;
	var panels:Null<Int>;
	var sheets:Null<Int>;
}

typedef MoldOrientationResult = {
	var rankMetrics:OrientationMetricTuple;
	var decidingReason:String;
	var id:String;
	var documentFingerprint:String;
	var planFingerprint:Null<String>;
	var planningSucceeded:Bool;
	var geometryChecked:Bool;
	var planningFailure:Null<String>;
	var sequence:Null<OrientationGeometryAssessment>;
	var fill:Null<FillReport>;
	var cure:Null<CureReport>;
	var sections:Null<OrthogonalSections.OrthogonalSectionReport>;
	var castability:Null<CastabilityAnalysis.CastabilityReport>;
	var thermalCoverage:Null<ThermalCoverage.ThermalCoverageReport>;
	var handling:Null<HandlingPlan>;
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
		var aUnsupported = a.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.feature.unsupported.")
			&& v.status == ToolingCheckStatus.Unsupported)
			.length > 0;
		var bUnsupported = b.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.feature.unsupported.")
			&& v.status == ToolingCheckStatus.Unsupported)
			.length > 0;
		if (aUnsupported != bUnsupported)
			return aUnsupported ? 1 : -1;
		if (a.geometryChecked != b.geometryChecked)
			return a.geometryChecked ? -1 : 1;
		for (value in [
			Reflect.compare(a.rankMetrics.failCount, b.rankMetrics.failCount),
			Reflect.compare(a.rankMetrics.openChangeRequests, b.rankMetrics.openChangeRequests),
			Reflect.compare(a.rankMetrics.trappedVolumeMm3, b.rankMetrics.trappedVolumeMm3),
			Reflect.compare(b.rankMetrics.worstCureMarginC == null ? Math.NEGATIVE_INFINITY : b.rankMetrics.worstCureMarginC,
				a.rankMetrics.worstCureMarginC == null ? Math.NEGATIVE_INFINITY : a.rankMetrics.worstCureMarginC),
			Reflect.compare(a.rankMetrics.requiresFlip ? 1 : 0, b.rankMetrics.requiresFlip ? 1 : 0),
			Reflect.compare(a.rankMetrics.panels == null ? 1e9 : a.rankMetrics.panels, b.rankMetrics.panels == null ? 1e9 : b.rankMetrics.panels),
			Reflect.compare(a.rankMetrics.sheets == null ? 1e9 : a.rankMetrics.sheets, b.rankMetrics.sheets == null ? 1e9 : b.rankMetrics.sheets)
		])
			if (value != 0)
				return value;
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

			if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(d.recipe))
				throw "Stale orientation process recipe";
			var plan:Null<CastingMoldPlan> = null,
				planningFailure:Null<String> = null;
			try
				plan = CastingMoldPlanner.plan(d, candidate.options)
			catch (error:Dynamic)
				planningFailure = Std.string(error);
			var shared:Null<CastingProcessReport> = null,
				processFailure:Null<String> = null;
			if (plan != null)
				try
					shared = CastingProcessAnalysis.analyze(plan, spec, fabrication.gravityMps2, fabrication)
				catch (error:Dynamic)
					processFailure = Std.string(error);
			var fill = shared == null ? null : shared.fill;
			var cure = shared == null ? null : shared.cure;
			var sections = shared == null ? null : shared.sections;
			var result:MoldOrientationResult = {
				id: candidate.id,
				documentFingerprint: fingerprint,
				planFingerprint: null,
				planningSucceeded: false,
				geometryChecked: false,
				planningFailure: planningFailure,
				rankMetrics: {
					failCount: 0,
					openChangeRequests: 0,
					trappedVolumeMm3: 0,
					worstCureMarginC: null,
					requiresFlip: false,
					panels: null,
					sheets: null
				},
				decidingReason: "not ranked",
				sequence: null,
				fill: fill,
				cure: cure,
				sections: sections,
				castability: shared == null ? null : shared.castability,
				thermalCoverage: shared == null ? null : shared.thermalCoverage,
				handling: plan == null ? null : plan.handling,
				effort: null,
				changeRequests: [],
				validation: shared == null ? [] : shared.validation.copy()
			};
			if (plan == null)
				result.validation.push(FillAnalysis.check("orientation.planner", [d.input.id], ToolingCheckStatus.Unsupported,
					"Mold planning failed: " + result.planningFailure, [fingerprint]));
			else {
				result.planningSucceeded = true;
				result.planFingerprint = CastingMoldCodec.fingerprint(plan);
				result.changeRequests = plan.changeRequests.copy();
				if (shared == null) {
					result.validation = result.validation.concat(plan.validation);
					result.validation.push(FillAnalysis.check("orientation.process", [d.input.id], ToolingCheckStatus.Fail,
						"Candidate process analysis failed: " + processFailure, [fingerprint]));
				}

				var fabricated:Null<FabricationPlan> = null;
				try
					fabricated = ToolingFabrication.plan(plan, fabrication, cure)
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
						result.geometryChecked = checks.filter(v -> StringTools.startsWith(v.rule, "tooling.cad.")).length == 4
							&& ["reconstruction", "coverage", "assembly-graph", "demolding-graph"].filter(name -> checks.filter(v -> v.rule == "tooling.cad."
								+ name
								&& v.status == ToolingCheckStatus.Pass)
								.length == 1)
								.length == 4;
					} catch (error:Dynamic)
						result.validation.push(FillAnalysis.check("orientation.geometry", [d.input.id], ToolingCheckStatus.Fail,
							"Candidate CAD/swept-graph assessment failed: " + Std.string(error), [result.planFingerprint]));
				}
			}
			if (result.castability != null)
				for (check in result.castability.validation)
					if (result.validation.filter(v -> v.rule == check.rule).length == 0)
						result.validation.push(check);
			var unique = new Map<String, ToolingValidation>();
			for (record in result.validation) {
				if (unique.exists(record.rule) && ToolingCodec.canonical(unique.get(record.rule)) != ToolingCodec.canonical(record))
					throw "Conflicting orientation validation: " + record.rule;
				unique.set(record.rule, record);
			}
			result.validation = [for (record in unique) record];
			var trapped = 0.0;
			if (fill != null)
				for (pocket in fill.trappedPockets)
					trapped += pocket.volumeMm3;
			var margin:Null<Float> = null, allLimits = true;
			if (cure != null)
				for (limit in cure.limits) {
					if (limit.marginC == null)
						allLimits = false;
					else
						margin = margin == null ? limit.marginC : Math.min(margin, limit.marginC);
				}
			result.rankMetrics = {
				failCount: count(result, ToolingCheckStatus.Fail),
				openChangeRequests: result.changeRequests.length,
				trappedVolumeMm3: trapped,
				worstCureMarginC: allLimits ? margin : null,
				requiresFlip: result.handling != null && result.handling.requiresFlip,
				panels: result.effort == null ? null : result.effort.panels,
				sheets: result.effort == null ? null : result.effort.sheetCount
			};
			result.validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
			results.push(result);
		}
		results.sort(compare);
		for (result in results) {
			var other = result == results[0] && results.length > 1 ? results[1] : results[0];
			var unsupported = result.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.feature.unsupported.")
				&& v.status == ToolingCheckStatus.Unsupported)
				.length > 0;
			var otherUnsupported = other.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.feature.unsupported.")
				&& v.status == ToolingCheckStatus.Unsupported)
				.length > 0;
			var reason = result.planningSucceeded != other.planningSucceeded ? "planning feasibility" : unsupported != otherUnsupported ? "supported feature strategies" : result.geometryChecked != other.geometryChecked ? "CAD assessment" : result.rankMetrics.failCount != other.rankMetrics.failCount ? "failure count" : result.rankMetrics.openChangeRequests != other.rankMetrics.openChangeRequests ? "open change requests" : result.rankMetrics.trappedVolumeMm3 != other.rankMetrics.trappedVolumeMm3 ? "trapped volume" : result.rankMetrics.worstCureMarginC != other.rankMetrics.worstCureMarginC ? "worst cure margin" : result.rankMetrics.requiresFlip != other.rankMetrics.requiresFlip ? "required flip" : result.rankMetrics.panels != other.rankMetrics.panels ? "panel count" : result.rankMetrics.sheets != other.rankMetrics.sheets ? "sheet count" : "stable ID after a metric tie";
			result.decidingReason = "Compared with "
				+ other.id
				+ ", decided by "
				+ reason
				+ (result.planningFailure == null ? "" : "; " + result.planningFailure);
		}

		var best = results[0],
			preferred = best.planningSucceeded && best.geometryChecked && count(best, ToolingCheckStatus.Fail) == 0 ? best.id : null;
		return {
			version: 5,
			sourceFingerprint: source,
			rankingRule: "feasibility gates: planned, supported feature strategies, CAD checked; then failure count, open change requests, trapped volume ascending, worst cure margin descending (unknown last), flip false first, panel count, sheet count, stable ID",
			rankedCandidates: results,
			preferredReviewCandidateId: preferred
		};
	}
}
