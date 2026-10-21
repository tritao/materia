package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingIds;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.CastingSetup.CoreStrategy;

enum abstract CastabilityAction(String) to String {
	var WidenPassage = "widen-passage";
	var ReduceSection = "reduce-section";
	var RouteVent = "route-vent";
	var ReviewCeiling = "review-ceiling-or-setup";
	var SplitOrSacrificial = "split-or-sacrificial";
	var QualifyBreakout = "qualify-breakout";
	var AnalyzeJunction = "analyze-junction";
	var DeclareLimit = "declare-process-limit";
	var DeclareLiftingPoint = "declare-lifting-point";
}

enum abstract ProposalOwner(String) to String {
	var DesignReview = "design-review";
	var ToolingReview = "tooling-review";
	var ProcessReview = "process-review";
}

/** A review alternative, never an applied edit or a claim of a routed vent.
 * Dimensions and locations are finished geometry in mold coordinates, mm.
 * Unknown parameter mappings remain explicit; the source design owns acceptance.
 */
typedef CastabilityProposal = {
	var id:String;
	var rule:String;
	var sourceIds:Array<String>;
	var action:CastabilityAction;
	var owner:ProposalOwner;
	var measuredMm:Null<Float>;
	var targetMm:Null<Float>;
	var axis:Null<SectionAxis>;
	var region:Null<MoldBox>;
	var point:Null<{x:Float, y:Float, z:Float}>;
	var instruction:String;
	var sourceParameterId:Null<String>;
	var proposedValue:Null<Float>;
}

typedef CastabilityReport = {
	var version:Int;
	var sourceFingerprint:String;
	var processFingerprint:String;
	var minimumOpposingFaceSpanMm:Float;
	var designMetrics:CastingLevelOne.CastingDesignMetrics;
	var heatRemovingInterval:Null<JunctionThickness.JunctionThicknessReport>;
	var maximumInscribedThicknessMm:Null<Float>;
	var maximumDiameterLowerMm:Float;
	var maximumDiameterUpperMm:Float;
	var validation:Array<ToolingValidation>;
	var proposals:Array<CastabilityProposal>;
}

/** Cheap screening over exact shared section/fill records. No CFD, adhesion,
 * geometry edits or duplicate sweep solver.
 */
class CastabilityAnalysis {
	static function sources(ids:Array<String>):Array<String> {
		var out:Array<String> = [];
		for (id in ids)
			if (out.indexOf(id) < 0)
				out.push(id);
		out.sort(Reflect.compare);
		return out;
	}

	static function proposal(rule:String, ids:Array<String>, action:CastabilityAction, owner:ProposalOwner, instruction:String, ?measured:Float,
			?target:Float, ?axis:SectionAxis, ?region:MoldBox, ?point:{
			x:Float,
			y:Float,
			z:Float
		}):CastabilityProposal {
		var record:CastabilityProposal = {
			id: "proposal",
			rule: rule,
			sourceIds: sources(ids),
			action: action,
			owner: owner,
			measuredMm: measured,
			targetMm: target,
			axis: axis,
			region: region,
			point: point,
			instruction: instruction,
			sourceParameterId: null,
			proposedValue: target
		};
		record.id = ToolingIds.derive("proposal-", haxe.crypto.Sha256.encode(ToolingCodec.canonical(record)).substr(0, 24));
		return record;
		}

	public static function analyze(document:ToolingDocument, raw:CastingProcessSpec, fill:FillReport, sections:OrthogonalSectionReport,
			expectedSourceFingerprint:String, expectedProcessFingerprint:String, ?plannerChecks:Array<ToolingValidation>):CastabilityReport {
		var d = ToolingCodec.snapshot(document),
			spec = ProcessSpecCodec.snapshot(raw);
		if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(d.recipe))
			throw "Stale castability recipe";
		if (sections.sourceFingerprint != expectedSourceFingerprint
			|| fill.sourceFingerprint != expectedSourceFingerprint
			|| sections.spans.length == 0)
			throw "Mismatched castability section/fill source";
		if (fill.processFingerprint != expectedProcessFingerprint)
			throw "Mismatched castability fill process";
		var checks:Array<ToolingValidation> = [],
			proposals:Array<CastabilityProposal> = [];
		var root = d.input.id, limits = spec.castability;
		var levelOne = CastingLevelOne.analyze(d);
		checks = checks.concat(levelOne.validation);
		proposals = proposals.concat(levelOne.proposals);
		var thermalInterval = ThermalFaces.interval(d, sections, spec);
		var spans = sections.spans.copy();
		spans.sort((a, b) -> a.thicknessMm != b.thicknessMm ? Reflect.compare(a.thicknessMm, b.thicknessMm) : Reflect.compare(a.id, b.id));
		var minimum = spans[0].thicknessMm;
		var minimumSpan = spans[0];
		var minimumOwners = sources(minimumSpan.lowerSourceIds.concat(minimumSpan.upperSourceIds));
		var minimumRule = "process.castability.minimum-passage";
		if (limits.maximumAggregateSizeMm == null || limits.minimumPassageAggregateRatio == null) {
			checks.push(FillAnalysis.check(minimumRule, minimumOwners, Pending,
				"Declare maximum aggregate size and minimum passage/aggregate ratio; narrowest opposing-face span is " + minimum + " mm.", limits.evidence));
			proposals.push(proposal(minimumRule, minimumOwners, DeclareLimit, ProcessReview,
				"Supply the mix grading and recipe passage ratio, then rerun the section screen.", minimum, null, minimumSpan.axis, minimumSpan.strips[0]));
		} else {
			var required = limits.maximumAggregateSizeMm * limits.minimumPassageAggregateRatio;
			var narrow = spans.filter(s -> s.thicknessMm < required - 1e-7);
			var ids = sources([for (s in narrow) for (id in s.lowerSourceIds.concat(s.upperSourceIds)) id]);
			checks.push(FillAnalysis.check(minimumRule, ids.length == 0 ? [root] : ids, narrow.length == 0 ? Pass : Fail,
				"Opposing-face passage screen: minimum "
				+ minimum
				+ " mm; declared aggregate-based minimum "
				+ required
				+ " mm. This is a geometry screen, not a prediction of paste flow or compaction.",
				limits.evidence));
			for (s in narrow)
				proposals.push(proposal(minimumRule, s.lowerSourceIds.concat(s.upperSourceIds), WidenPassage, DesignReview,
					"Increase separation of these source faces to at least the target, or select a finer qualified aggregate mix. Recheck stiffness and tooling.",
					s.thicknessMm, required, s.axis, s.strips[0]));
		}
		var maximumRule = "process.castability.maximum-section",
			maximum = sections.maximumInscribedThicknessMm;
		if (limits.maximumSectionThicknessMm == null) {
			checks.push(FillAnalysis.check(maximumRule, [root], Pending, "No recipe maximum section thickness declared; cure checks remain independent.",
				limits.evidence));
			proposals.push(proposal(maximumRule, [root], DeclareLimit, ProcessReview,
				"Declare a section screening limit with material/thermal evidence; a limit alone does not qualify cure."));
		} else {
			var bound = thermalInterval;
			if (bound == null) {
				checks.push(FillAnalysis.check(maximumRule, [root], Unsupported, "No heat-removing face is available for a thermal section interval.", []));
			} else {
				var threshold:Float = cast limits.maximumSectionThicknessMm;
				var fails = bound.lowerDiameterMm > threshold + 1e-7;
				var passes = bound.upperDiameterMm <= threshold + 1e-7;
				checks.push(FillAnalysis.check(maximumRule, bound.witnessSourceIds, fails ? Fail : passes ? Pass : Pending,
					"Heat-removing-face diameter is bounded by "
					+ bound.lowerDiameterMm
					+ ".."
					+ bound.upperDiameterMm
					+ " mm against declared limit "
					+ limits.maximumSectionThicknessMm
					+ " mm. Geometry does not replace cure analysis.",
					limits.evidence));
				if (fails)
					proposals.push(proposal(maximumRule, bound.witnessSourceIds, ReduceSection, DesignReview,
						"Add or enlarge a pocket near this witness to reduce the section, or qualify a different cure process. Recheck structural performance.",
						bound.lowerDiameterMm, limits.maximumSectionThicknessMm, null, null, bound.witness));
				else if (!passes)
					proposals.push(proposal(maximumRule, bound.witnessSourceIds, AnalyzeJunction, ProcessReview,
						"Refine the inscribed-diameter interval until it resolves this limit; no midpoint acceptance.", null,
						limits.maximumSectionThicknessMm, null, null, bound.witness));
			}
		}
		// Keep the exact fill rule/status/ownership: suggestions do not clear it.
		for (check in fill.validation) {
			checks.push(check);
			if (check.rule == "process.fill.air-escape-width" && check.status != Pass)
				proposals.push(proposal(check.rule, check.affectedIds, check.status == Pending ? DeclareLimit : ReviewCeiling,
					check.status == Pending ? ProcessReview : ToolingReview,
					check.status == Pending ? "Declare the recipe ceiling air-escape-width limit and rerun filling checks." : "Evaluate another setup or design a vent route for these ceilings; verify its access and seal before replanning."));
		}
		for (pocket in fill.trappedPockets)
			proposals.push(proposal("process.fill.trap." + pocket.id, pocket.sourceIds, RouteVent, ToolingReview,
				"Route a continuous vent from this candidate ceiling point to atmosphere, with a sealing/plug plan; if it crosses casting material, request a source change. Rerun topology and CAD.",
				null, null, null, null, pocket.proposedVentPoint));
		if (maximum == null && limits.maximumSectionThicknessMm == null)
			proposals.push(proposal(maximumRule, [root], AnalyzeJunction, ProcessReview,
				"Junction diameter bounds are available. Resolve junction heat flow and thermal coverage before accepting section/cure limits."));
		var strategies = materia.tooling.casting.CastingStrategies.resolve(d);
		strategies.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
		var reportedParents = new Map<String, Bool>();
		for (strategy in strategies) {
			var source = d.input.features.filter(feature -> feature.id == strategy.featureId)[0];
			var parent = source.derivedFrom == null ? source.id : source.derivedFrom;
			if (reportedParents.exists(parent))
				continue;
			reportedParents.set(parent, true);
			var rule = "process.castability.release." + parent;
			if (strategy.strategy == SacrificialEPS) {
				checks.push(FillAnalysis.check(rule, [parent], Pending,
					"EPS breakout accessibility, fragment size and release need a physical trial; a rigid-member sweep does not establish breakup access.",
					[]));
				proposals.push(proposal(rule, [parent], QualifyBreakout, ToolingReview,
					"Trial breakout through the declared opening after fixture release; record access, remaining foam and damage against this source revision."));
			} else if (strategy.strategy == ReusableFormer) {
				var draftRule = "tooling.former.draft." + strategy.featureId;
				var siblings = d.input.features.filter(feature -> (feature.derivedFrom == null ? feature.id : feature.derivedFrom) == parent);
				var draftRules = [for (feature in siblings) "tooling.former.draft." + feature.id];
				var actual = plannerChecks == null ? [] : plannerChecks.filter(v -> draftRules.indexOf(v.rule) >= 0);
				if (actual.length > 0) {
					for (check in actual)
						checks.push(check);
					if (siblings.length == 1)
						rule = draftRule;
					else {
						var status = actual.filter(v -> v.status == Unsupported).length > 0 ? Unsupported
							: actual.filter(v -> v.status == Fail).length > 0 ? Fail
							: actual.length < siblings.length || actual.filter(v -> v.status == Pending).length > 0 ? Pending : Pass;
						checks.push(FillAnalysis.check(rule, [parent], status,
							"Release qualification covers every fragment of the authored core unit.", []));
					}
				} else
					checks.push(FillAnalysis.check(rule, [parent], Pending,
						"Run the real planner and exact swept removal checks for rigid-former extraction; adhesion needs physical evidence.", []));
				if (actual.length == 0 || actual.filter(v -> v.status != Pass).length > 0)
					proposals.push(proposal(rule, [parent], SplitOrSacrificial, DesignReview,
						"This planner cannot realize drafted or split formers. Only supported fill-open EPS or a source change to supported release geometry clears this request; rerun planning and exact CAD checks."));
			} else {
				checks.push(FillAnalysis.check(rule, [parent], Unsupported, "Feature has no supported tooling strategy.", []));
				proposals.push(proposal(rule, [parent], SplitOrSacrificial, ToolingReview,
					"Select an explicit supported tooling strategy and rerun planning before designing its release."));
			}
		}
		checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		proposals.sort((a, b) -> Reflect.compare(a.id, b.id));
		return {
			version: 3,
			sourceFingerprint: fill.sourceFingerprint,
			processFingerprint: fill.processFingerprint,
			minimumOpposingFaceSpanMm: minimum,
			heatRemovingInterval: thermalInterval,
			designMetrics: levelOne.metrics,
			maximumInscribedThicknessMm: maximum,
			maximumDiameterLowerMm: sections.junction.lowerDiameterMm,
			maximumDiameterUpperMm: sections.junction.upperDiameterMm,
			validation: checks,
			proposals: proposals
		};
	}
}
